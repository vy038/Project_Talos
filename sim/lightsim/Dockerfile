FROM node:20-bullseye

RUN apt-get update && apt-get install -y \
    cmake \
    make \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Build context is the Project_Talos repo root
COPY sim/lightsim/ ./lightsim/
COPY firmware/ ./firmware/

# Build the C simulator binary
RUN mkdir -p lightsim/build && \
    cd lightsim/build && \
    cmake .. && \
    make -j$(nproc)

# Install Node.js server deps
RUN cd lightsim/server && npm install

EXPOSE 3000

WORKDIR /app/lightsim/server
CMD ["node", "server.js"]
