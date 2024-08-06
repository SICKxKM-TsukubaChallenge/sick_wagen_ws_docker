FROM ubuntu:focal-20210609 AS builder

ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

RUN apt-get update
RUN apt-get install -y curl
RUN apt-get install -y --no-install-recommends gcc libc-dev
RUN curl -o /usr/local/bin/su-exec.c https://raw.githubusercontent.com/ncopa/su-exec/master/su-exec.c
RUN gcc -Wall /usr/local/bin/su-exec.c -o /usr/local/bin/su-exec
RUN chown root:root /usr/local/bin/su-exec
RUN chmod 0755 /usr/local/bin/su-exec

FROM ubuntu:focal-20210609
LABEL maintainer="Daisuke Sato <tiryoh@gmail.com>"

COPY --from=builder /usr/local/bin/su-exec /sbin/
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends keyboard-configuration language-pack-en && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends wget curl git build-essential ca-certificates tzdata tmux gnupg2 zip unzip \
        vim sudo lsb-release locales bash-completion zsh iproute2 iputils-ping net-tools dnsutils terminator && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends ros-noetic-desktop-full python3-rosdep && \
    apt-get install -y --no-install-recommends python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools python3-osrf-pycommon python3-vcstool python3-pip && \
    apt-get install -y --no-install-recommends ros-noetic-teleop-twist-keyboard && \
    apt-get install -y --no-install-recommends ros-noetic-navigation && \
    apt-get install -y --no-install-recommends ros-noetic-sick-scan-xd && \
    apt-get install -y --no-install-recommends ros-noetic-robot-localization && \
    apt-get install -y --no-install-recommends ros-noetic-robot-state-publisher && \
    apt-get install -y --no-install-recommends ros-noetic-mcl-3dl && \
    apt-get install -y --no-install-recommends ros-noetic-serial && \
    apt-get install -y --no-install-recommends ros-noetic-ublox && \
    apt-get install -y --no-install-recommends ros-noetic-usb-cam && \
    apt-get install -y --no-install-recommends ros-noetic-mcl-3dl-msgs && \
    apt-get install -y --no-install-recommends ros-noetic-ecl-core && \
    apt-get install -y --no-install-recommends ros-noetic-move-base && \
    apt-get install -y --no-install-recommends ros-noetic-joy && \
    apt-get install -y --no-install-recommends ros-noetic-gmapping && \
    apt-get install -y --no-install-recommends ros-noetic-map-server && \
    apt-get install -y --no-install-recommends ros-noetic-amcl && \
    apt-get install -y --no-install-recommends ros-noetic-dwa-local-planner && \
    apt-get install -y --no-install-recommends ros-noetic-tf2-sensor-msgs && \
    apt-get install -y --no-install-recommends ros-noetic-trajectory-tracker && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install whill

# RUN wget -O /tmp/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.3.zip
# RUN cd /tmp && \
#     unzip gtsam.zip -d /tmp/ && \
#     cd gtsam-4.0.3/ && \
#     mkdir build && \
#     cd build && \
#     cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && \
#     make install
 
# RUN python3 -m pip install opencv-python torch torchvision onnxruntime
# RUN python3 -m pip install numpy --upgrade --ignore-installed
# RUN python3 -m pip install moteus_gui

RUN rosdep init && rosdep update

COPY ros_entrypoint.sh /ros_entrypoint.sh
# エントリポイントスクリプトに実行権限を付与
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
