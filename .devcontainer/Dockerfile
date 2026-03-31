FROM ghcr.io/bounverif/esmini:latest-devel

# Create non-root user (base image is AlmaLinux 8 without an 'ubuntu' user)
RUN (getent group 1000 || groupadd -g 1000 ubuntu) \
    && (getent passwd 1000 || useradd -m -s /bin/bash -u 1000 -g 1000 ubuntu)

# Install vcpkg prerequisites and dev libraries for full build
# - zip: required by vcpkg
# - lz4-devel, libzstd-devel: required by asam-osi-utilities (Phase A)
# - libX11-devel, libXrandr-devel, libXinerama-devel, mesa-libGL-devel,
#   fontconfig-devel: required for OSG/ImPlot linking (Phase B)
RUN dnf install -y -q \
        zip \
        lz4-devel libzstd-devel \
        libX11-devel libXrandr-devel libXinerama-devel \
        mesa-libGL-devel fontconfig-devel \
    && dnf clean all

# Bootstrap vcpkg for OSI two-phase build
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git /opt/vcpkg \
    && /opt/vcpkg/bootstrap-vcpkg.sh -disableMetrics \
    && ln -s /opt/vcpkg/vcpkg /usr/local/bin/vcpkg

ENV VCPKG_ROOT=/opt/vcpkg

# Note: The base image ships protobuf 3.21.12 at /opt/bazalt/.
# Use the "base" preset (not "vcpkg") for Phase A inside this container
# to avoid version conflicts with vcpkg's newer protobuf.
# Example:
#   cmake -S externals/asam-osi-utilities -B build-deps --preset base \
#     -DCMAKE_PREFIX_PATH=/opt/bazalt \
#     -DProtobuf_PROTOC_EXECUTABLE=/opt/bazalt/bin/protoc \
#     -DCMAKE_CXX_FLAGS="-fpermissive"
