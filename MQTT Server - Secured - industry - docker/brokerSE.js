'use strict';

const cluster = require('cluster');
const os = require('os');
const fs = require('fs');
const path = require('path');
const net = require('net');
const tls = require('tls');
const express = require('express');
const promClient = require('prom-client');
const pino = require('pino')({ level: process.env.LOG_LEVEL || 'info' });

const { createClient } = require('redis');
const MQEMITTER_REDIS = require('mqemitter-redis');
const aedesPersistenceRedis = require('aedes-persistence-redis');
const Aedes = require('aedes');

require('dotenv').config();

// ---------------- Config ----------------
const REDIS_URL = process.env.REDIS_URL || 'redis://127.0.0.1:6379';
const TLS_PORT = parseInt(process.env.MQTT_TLS_PORT || '8883', 10);
const TCP_PORT = parseInt(process.env.MQTT_PORT || '1883', 10);
const HTTP_PORT = parseInt(process.env.HTTP_PORT || '3001', 10);
const WORKERS = parseInt(process.env.WORKERS || Math.min(os.cpus().length, 4), 10);

const MAX_INFLIGHT = parseInt(process.env.MAX_INFLIGHT || '100', 10);
const CLIENT_PUB_RATE = parseInt(process.env.CLIENT_PUB_RATE || '200', 10);
const CLIENT_SUBS_LIMIT = parseInt(process.env.CLIENT_SUBS_LIMIT || '1000', 10);
const TOKEN_BUCKET_INTERVAL = parseInt(process.env.TOKEN_BUCKET_INTERVAL || '500', 10);

const TLS_CERT_PATH = process.env.TLS_CERT ? path.resolve(__dirname, process.env.TLS_CERT) : null;
const TLS_KEY_PATH = process.env.TLS_KEY ? path.resolve(__dirname, process.env.TLS_KEY) : null;

// ---------------- Helpers ----------------
function parseRedisUrl(url) {
  try {
    const u = new URL(url);
    return {
      protocol: u.protocol.replace(':', ''),
      host: u.hostname,
      port: parseInt(u.port || '6379', 10),
      username: u.username || undefined,
      password: u.password || undefined,
      url // keep original for modules that accept url
    };
  } catch (e) {
    pino.warn('Invalid REDIS_URL, falling back to localhost:6379');
    return { host: '127.0.0.1', port: 6379, url: 'redis://127.0.0.1:6379' };
  }
}

const redisCfg = parseRedisUrl(REDIS_URL);

// ---------------- Redis wait helper ----------------
async function waitForRedis(url) {
  const client = createClient({ url });
  client.on('error', (err) => pino.warn(`Redis connection error (waitForRedis): ${err.message}`));
  try {
    await client.connect();
    await client.ping();
    await client.quit();
    pino.info('Redis connection successful (waitForRedis)');
    return true;
  } catch (err) {
    pino.warn(`Redis connection failed (waitForRedis): ${err.message}`);
    try { await client.quit(); } catch (_) {}
    return false;
  }
}

// ---------------- global error handlers ----------------
process.on('uncaughtException', (err) => {
  pino.error({ err }, 'uncaughtException - worker will exit');
  // allow cluster master to restart worker by exiting
  process.exit(1);
});
process.on('unhandledRejection', (reason) => {
  pino.error({ reason }, 'unhandledRejection');
});

// ---------------- Master ----------------
if (cluster.isMaster) {
  pino.info(`[master] starting ${WORKERS} worker(s)`);
  for (let i = 0; i < WORKERS; i++) cluster.fork();

  cluster.on('exit', (worker, code, signal) => {
    pino.warn(`[master] worker ${worker.process.pid} died (code=${code} signal=${signal}). Restarting...`);
    cluster.fork();
  });

} else {
  (async () => {
    // Wait for Redis to be available before creating MQ emitter/persistence
    let redisReady = false;
    let attempts = 0;
    const maxAttempts = 20;
    while (!redisReady && attempts < maxAttempts) {
      attempts++;
      redisReady = await waitForRedis(redisCfg.url);
      if (!redisReady) {
        pino.warn(`Waiting for Redis... (attempt ${attempts}/${maxAttempts})`);
        await new Promise((r) => setTimeout(r, 2000));
      }
    }
    if (!redisReady) {
      pino.error('Failed to connect to Redis after max attempts, exiting worker');
      process.exit(1);
    }

    // ---------------- Create MQ emitter & persistence (explicit host/port & url) ----------------
    let mq;
    let persistence;
    try {
      // Some versions accept url, some prefer host/port — provide both
      const mqOptions = { url: redisCfg.url, host: redisCfg.host, port: redisCfg.port };
      mq = MQEMITTER_REDIS(mqOptions);
      // attach error handler so 'error' event doesn't crash process
      if (mq && typeof mq.on === 'function') {
        mq.on('error', (err) => pino.error({ err }, '[mqemitter] error'));
      }
    } catch (err) {
      pino.error({ err }, 'Failed to create MQ emitter (mqemitter-redis)');
      throw err;
    }

    try {
      const persistenceOptions = { url: redisCfg.url, host: redisCfg.host, port: redisCfg.port };
      persistence = aedesPersistenceRedis(persistenceOptions);
      if (persistence && typeof persistence.on === 'function') {
        persistence.on('error', (err) => pino.error({ err }, '[persistence] error'));
      }
    } catch (err) {
      pino.error({ err }, 'Failed to create persistence (aedes-persistence-redis)');
      throw err;
    }

    // ---------------- Aedes broker ----------------
    const aedes = Aedes({
      persistence,
      mq,
      concurrency: parseInt(process.env.AEDES_CONCURRENCY || '1000', 10),
      heartbeatInterval: parseInt(process.env.HEARTBEAT_INTERVAL_MS || '60000', 10)
    });

    // ---------------- Rate Limiting (token bucket) ----------------
    const tokenBuckets = new Map();
    function getBucket(clientId) {
      if (!tokenBuckets.has(clientId)) tokenBuckets.set(clientId, { tokens: CLIENT_PUB_RATE, last: Date.now() });
      return tokenBuckets.get(clientId);
    }
    setInterval(() => {
      const now = Date.now();
      for (const [id, b] of tokenBuckets) {
        const elapsed = (now - b.last) / 1000;
        if (elapsed <= 0) continue;
        b.tokens = Math.min(CLIENT_PUB_RATE, b.tokens + elapsed * CLIENT_PUB_RATE);
        b.last = now;
        // optional GC: remove idle buckets after X minutes
        // if ((now - b.last) > 1000 * 60 * 30) tokenBuckets.delete(id);
      }
    }, TOKEN_BUCKET_INTERVAL);

    // ---------------- Auth & ACL ----------------
    aedes.authenticate = (client, username, password, callback) => {
      try {
        const pw = password ? password.toString() : '';
        if (process.env.MQTT_USER && process.env.MQTT_PASS) {
          if (username === process.env.MQTT_USER && pw === process.env.MQTT_PASS) return callback(null, true);
        }
        const err = new Error('Auth failed');
        err.returnCode = 4;
        return callback(err, false);
      } catch (err) {
        pino.error({ err }, 'authenticate error');
        return callback(err, false);
      }
    };

    aedes.authorizePublish = (client, packet, cb) => {
      try {
        const bucket = getBucket(client.id);
        if (bucket.tokens < 1) return cb(new Error('Rate limit exceeded'));
        bucket.tokens -= 1;

        if (client && client.inflight && client.inflight >= MAX_INFLIGHT) return cb(new Error('Too many inflight messages'));
        if (packet.payload && packet.payload.length > parseInt(process.env.MAX_PAYLOAD || '1048576')) return cb(new Error('Payload too large'));
        return cb(null);
      } catch (e) {
        pino.error({ e }, 'authorizePublish error');
        return cb(e);
      }
    };

    aedes.authorizeSubscribe = (client, sub, callback) => {
      try {
        if (client && client.subscriptions && Object.keys(client.subscriptions).length >= CLIENT_SUBS_LIMIT) {
          return callback(new Error('Subscription limit reached'));
        }
        return callback(null, sub);
      } catch (e) {
        pino.error({ e }, 'authorizeSubscribe error');
        return callback(e);
      }
    };

    // ---------------- Logging ----------------
    aedes.on('client', (c) => pino.info(`[broker] client connected: ${c ? c.id : 'unknown'}`));
    aedes.on('clientDisconnect', (c) => pino.info(`[broker] client disconnected: ${c ? c.id : 'unknown'}`));
    // avoid logging every publish in high throughput env (debug level)
    aedes.on('publish', (pkt, c) => {
      if (c) pino.debug(`[broker] publish from ${c.id} topic=${pkt.topic} qos=${pkt.qos}`);
    });
    aedes.on('error', (err) => pino.error({ err }, '[broker] aedes error'));

    // ---------------- TLS Server ----------------
    if (TLS_CERT_PATH && TLS_KEY_PATH) {
      try {
        const tlsOptions = {
          key: fs.readFileSync(TLS_KEY_PATH),
          cert: fs.readFileSync(TLS_CERT_PATH),
          maxVersion: 'TLSv1.2',
          requestCert: false,
          rejectUnauthorized: false,
          allowHalfOpen: true
        };
        // create TLS server with reusePort (Node 16+)
        const tlsServer = tls.createServer({ ...tlsOptions, reusePort: true }, aedes.handle);
        tlsServer.listen(TLS_PORT, () => pino.info(`[broker] TLS MQTT listening on ${TLS_PORT}`));
        tlsServer.on('error', (e) => pino.error({ e }, '[broker] TLS server error'));
      } catch (err) {
        pino.error({ err }, '[broker] failed to start TLS server - check cert/key paths');
      }
    } else {
      pino.warn('[broker] TLS_CERT or TLS_KEY missing - TLS server not started');
    }

    // ---------------- TCP Server ----------------
    if (process.env.ENABLE_TCP === 'true') {
      const tcpServer = net.createServer({ allowHalfOpen: true, pauseOnConnect: false }, aedes.handle);
      // `reusePort` isn't accepted here as option to net.createServer in some Node versions; using default.
      tcpServer.listen(TCP_PORT, () => pino.info(`[broker] TCP MQTT listening on ${TCP_PORT}`));
      tcpServer.on('error', (e) => pino.error({ e }, '[broker] TCP server error'));
    }

    // ---------------- HTTP Metrics ----------------
    const app = express();
    promClient.collectDefaultMetrics({ prefix: 'aedes_' });
    const connectedClients = new promClient.Gauge({ name: 'aedes_connected_clients', help: 'connected clients' });
    const totalPub = new promClient.Counter({ name: 'aedes_total_publishes', help: 'total publishes' });

    setInterval(() => connectedClients.set(aedes.connectedClients || 0), 1000);
    aedes.on('publish', () => totalPub.inc());

    app.get('/status', (req, res) => res.json({ broker: 'running', clients: aedes.connectedClients, pid: process.pid, uptime: process.uptime() }));
    app.get('/metrics', async (req, res) => {
      res.set('Content-Type', promClient.register.contentType);
      res.end(await promClient.register.metrics());
    });

    app.listen(HTTP_PORT, () => pino.info(`[http] status/metrics listening on ${HTTP_PORT}`));

    pino.info('[worker] broker ready');

  })().catch(err => {
    pino.error('[worker] fatal error (top-level)', { error: err && err.message, stack: err && err.stack });
    process.exit(1);
  });
}
