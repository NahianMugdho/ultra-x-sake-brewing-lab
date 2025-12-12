nodemon brokerSE.js

then run redis in docker

(You can start Redis in docker:

✅ OPTION 1 — Start Redis using Docker (most common & easiest)

If you have Docker installed, run this:

docker run -d --name redis -p 6379:6379 redis

✔️ What this does:

downloads Redis

runs it in the background

exposes port 6379

makes it available to your local apps

Check if running:
docker ps


You should see redis running.)