# PX4 cross build for the NyxHop Zynq boards: Debian bullseye's armhf toolchain (gcc 10, glibc
# 2.31 - the same as Kuiper 2023_r2's on the board, so the binary runs there as it is).
FROM debian:bullseye
ENV DEBIAN_FRONTEND=noninteractive
# (bullseye's LTS is over: its packages are on the archive now)
RUN echo "deb http://archive.debian.org/debian bullseye main" > /etc/apt/sources.list \
    && echo "deb http://archive.debian.org/debian-security bullseye-security main" >> /etc/apt/sources.list \
    && rm -f /etc/apt/sources.list.d/* \
    && echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99archive
RUN apt-get update && apt-get install -y --no-install-recommends \
        crossbuild-essential-armhf gcc g++ cmake ninja-build make git ca-certificates file \
        python3 python3-pip python3-setuptools python3-wheel python3-numpy python3-lxml \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir \
        "empy>=3.3,<4" "jinja2>=2.8" pyyaml jsonschema kconfiglib pyros-genmsg packaging \
        "toml>=0.9" future psutil pyserial six argcomplete cerberus lark pygments pexpect
RUN git config --global --add safe.directory '*'
WORKDIR /src
