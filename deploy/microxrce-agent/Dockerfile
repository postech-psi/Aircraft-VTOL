FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends \
    git \
    cmake \
    g++ \
    make \
    ca-certificates \
    libasio-dev \
    libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    git clone --depth 1 -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j1 && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

CMD ["MicroXRCEAgent", "udp4", "-p", "8888"]
