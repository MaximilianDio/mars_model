FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

# Install additional packages
RUN apt-get update && apt-get install -y libgtest-dev
