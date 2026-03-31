FROM ghcr.io/bounverif/esmini:latest-devel

# Create non-root user (base image is AlmaLinux 8 without an 'ubuntu' user)
RUN (getent group 1000 || groupadd -g 1000 ubuntu) \
    && (getent passwd 1000 || useradd -m -s /bin/bash -u 1000 -g 1000 ubuntu)

# Install vcpkg prerequisites missing from base image
RUN dnf install -y -q zip && dnf clean all

# Bootstrap vcpkg for OSI two-phase build
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git /opt/vcpkg \
    && /opt/vcpkg/bootstrap-vcpkg.sh -disableMetrics \
    && ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg

ENV VCPKG_ROOT=/opt/vcpkg
