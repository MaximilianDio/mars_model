FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

SHELL ["/bin/bash", "-c"]

RUN apt update && apt-get install -y locales &&\
    locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV NO_AT_BRIDGE=1

## Essentials
RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    cmake \
    gdb \
    wget \
    git \
    curl \
    nano \
    llvm\
    libgtest-dev\
    && apt-get clean

ENV HOME=/home
## construct direcotry for dependencies
RUN cd ${HOME} &&\
    mkdir workspace
ENV DEP_DIR ${HOME}/dependencies

## install eigen
RUN apt update && apt install -y libeigen3-dev libboost-all-dev liburdfdom-dev && apt-get clean

## install pinocchio
RUN apt install -qqy lsb-release curl &&\
    mkdir -p /etc/apt/keyrings &&\
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc &&\
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list &&\
    apt update

RUN apt install -qqy robotpkg-py3*-pinocchio

ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH 
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH