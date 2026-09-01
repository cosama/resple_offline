ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        libeigen3-dev \
        libomp-dev \
        libpcl-dev \
        ninja-build \
        patch \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY core /src/core
COPY upstream /src/upstream

RUN python3 -m venv /opt/build-venv \
    && /opt/build-venv/bin/pip install --no-cache-dir --upgrade pip wheel \
    && /opt/build-venv/bin/pip wheel --no-cache-dir --wheel-dir /wheelhouse ./core

FROM ubuntu:${UBUNTU_VERSION} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/opt/resple/bin:${PATH}
ENV PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgomp1 \
        libpcl-common1.14 \
        libpcl-filters1.14 \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /wheelhouse /wheelhouse

RUN python3 -m venv /opt/resple \
    && /opt/resple/bin/pip install --no-cache-dir --no-index --find-links=/wheelhouse resple-offline \
    && rm -rf /wheelhouse

WORKDIR /data

CMD ["python", "-c", "import resple; print('resple-offline ready')"]
