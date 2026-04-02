FROM node:20-bullseye

RUN apt-get update && apt-get install -y \
    cmake \
    make \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Build context is the Project_Talos repo root
COPY sim/lightsim/ ./sim/lightsim/
COPY firmware/ ./firmware/

# Build the C simulator binary
RUN mkdir -p sim/lightsim/build && \
    cd sim/lightsim/build && \
    cmake .. && \
    make -j$(nproc)

# Install Node.js server deps
RUN cd sim/lightsim/server && npm install

# Fix permissions for Hugging Face user 1000
RUN chmod -R 777 /app

ENV PORT=7860
EXPOSE 7860

WORKDIR /app/sim/lightsim/server
CMD ["node", "server.js"]
