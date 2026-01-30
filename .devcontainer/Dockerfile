FROM osrf/ros:foxy-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
# Check if user exists first to avoid errors if the base image already has it
RUN if ! id -u $USERNAME > /dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    # [Optional] Add sudo support. Omit if you don't want to install sudo commands
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install handy tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    bash-completion \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL /bin/bash

# Source ROS setup in .bashrc for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME/ws
