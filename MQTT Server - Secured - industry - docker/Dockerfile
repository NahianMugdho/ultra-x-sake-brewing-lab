# Base image
FROM node:20-alpine

# Create app directory
WORKDIR /app

# Copy package.json first for caching
COPY package*.json ./

# Install dependencies
RUN npm install --production

# Copy all files
COPY . .

# Expose ports
EXPOSE 8883 1883 3001

# Run broker
CMD ["node", "brokerSE.js"]
