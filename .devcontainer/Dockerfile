FROM ghcr.io/bounverif/esmini:latest-devel

# Create non-root user (base image is AlmaLinux 8 without an 'ubuntu' user)
RUN (getent group 1000 || groupadd -g 1000 ubuntu) \
    && (getent passwd 1000 || useradd -m -s /bin/bash -u 1000 -g 1000 ubuntu)
