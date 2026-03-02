# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# System dependencies:
# - python3, venv, pip
# - libegl1, libgl1, libglib2.0-0       for Open3D on Ubuntu 24.04
# - python3-openvdb                       provides the pyopenvdb module for Python 3.12
#                                         (no PyPI wheel exists for Python 3.12)
# - libopenvdb-dev, libtbb-dev,
#   libboost-iostreams-dev, zlib1g-dev,
#   libblosc-dev                          runtime link deps for python3-openvdb
RUN apt-get update && apt-get install -y \
    python3 \
    python3-venv \
    python3-pip \
    libegl1 \
    libgl1 \
    libglib2.0-0 \
    python3-openvdb \
    libopenvdb-dev \
    libtbb-dev \
    libboost-iostreams-dev \
    zlib1g-dev \
    libblosc-dev \
    && rm -rf /var/lib/apt/lists/*

# --system-site-packages exposes the apt-installed python3-openvdb inside the
# venv without a pip install. This is required because no pyopenvdb wheel
# exists for Python 3.12 on PyPI.
RUN python3 -m venv /opt/venv --system-site-packages

# Use venv Python and pip
ENV PATH="/opt/venv/bin:$PATH"

# Install remaining Python dependencies (pyopenvdb intentionally omitted —
# it is sourced from the apt python3-openvdb package above)
RUN pip install --no-cache-dir \
    rosbags \
    open3d \
    numpy \
    scipy \
    tqdm

# Copy script
COPY bag_to_vdb.py .

RUN chmod +x bag_to_vdb.py

ENTRYPOINT ["/opt/venv/bin/python", "/app/bag_to_vdb.py"]

