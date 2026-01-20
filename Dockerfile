# syntax=docker/dockerfile:1
FROM nvidia/cuda:13.1.0-devel-ubuntu24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fluidsim
COPY . .

ARG CMAKE_CUDA_ARCHITECTURES=89

RUN set -eux; \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}; \
    cmake --build build -j

CMD ["./build/fluidsim", "--backend=cpu"]
