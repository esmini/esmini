# Dockerfile
# interactive/non-interactive: xhost +local:root
# interactive docker: docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --rm -v .:/mnt/esminiMainFolder:z --shm-size=2g ubuntu:22.04 /bin/bash
# interactive podman: podman run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --security-opt label=type:container_runtime_t --rm -v .:/mnt/esminiMainFolder:z --shm-size=2g ubuntu:22.04 /bin/bash
# non-interactive docker: docker build -t esmini_ubuntu:22.04 resources/dockers/;docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --rm -v .:/mnt/esminiMainFolder:z  --shm-size=2g esmini_ubuntu:22.04
# non-interactive podman: podman build -t esmini_ubuntu:22.04 resources/dockers/;podman run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --security-opt label=type:container_runtime_t --rm -v .:/mnt/esminiMainFolder:z  --shm-size=2g esmini_ubuntu:22.04
FROM ubuntu:22.04

# interactive: apt update;apt install --assume-yes build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black
RUN apt update
RUN apt install --assume-yes build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black

# interactive: cd /mnt/esminiMainFolder
WORKDIR /mnt/esminiMainFolder

# interactive: mkdir -p build;cd build;rm -rf *;cmake ..;cmake --build . --config Release --target install
RUN mkdir -p build && cd build && rm -rf *
WORKDIR /mnt/esminiMainFolder/build
RUN cmake ..
RUN cmake --build . --config Release --target install

# interactive: cd ..
WORKDIR /mnt/esminiMainFolder
CMD ["/bin/bash"]
