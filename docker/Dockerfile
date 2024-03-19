ARG DEBIAN_FRONTEND=noninteractive

FROM ubuntu:22.04 as python_dependencies
ARG DEBIAN_FRONTEND
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3.10 \
  python3.10-dev \
  python3.10-venv \
  python3-pip \
  python3-wheel \
  build-essential

FROM ros:humble-ros-base-jammy as dependencies
ARG DEBIAN_FRONTEND
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -y --no-install-recommends \
  openssh-client \
  libeigen3-dev \
  ros-humble-rmw-cyclonedds-cpp \
  curl \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

FROM dependencies as base
ARG DEBIAN_FRONTEND
ARG UID=1000
ARG GID=1000
ARG USERNAME=ros
RUN groupadd --gid $GID $USERNAME \
  && useradd -s /bin/bash --uid $UID --gid $GID -m $USERNAME -G audio,dialout,plugdev,sudo \
  && passwd -d $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN mkdir -p /etc/${USERNAME} \
  && chown -R ${USERNAME}:${USERNAME} /etc/${USERNAME} \
  && chown -R ${USERNAME}:${USERNAME} /usr/local
COPY --chown=$UID:$GID environment/bashrc_default /home/ros/.bashrc

FROM base as final
COPY --from=python_dependencies /usr/bin/python3.10 /usr/bin/python3.10
WORKDIR /home/ros
USER ros
