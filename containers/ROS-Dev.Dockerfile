FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y dos2unix

COPY ./scripts/install-build-essential.sh /scripts/
RUN dos2unix /scripts/install-build-essential.sh && chmod +x /scripts/install-build-essential.sh \
    && /scripts/install-build-essential.sh

# 3. Only copy and run cmake
COPY ./scripts/install-cmake.sh /scripts/
RUN dos2unix /scripts/install-cmake.sh && chmod +x /scripts/install-cmake.sh \
    && /scripts/install-cmake.sh

# 4. Only copy and run ROS2 (This is the longest step; keep it isolated!)
COPY ./scripts/install-ros2.sh /scripts/
RUN dos2unix /scripts/install-ros2.sh && chmod +x /scripts/install-ros2.sh \
    && /scripts/install-ros2.sh

RUN dos2unix /scripts/*.sh && chmod +x /scripts/*.sh

RUN apt-get update && apt-get install -y ros-humble-phidgets-drivers

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME --shell /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN usermod -aG dialout ${USERNAME}
USER $USERNAME

RUN mkdir -p /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

USER root
COPY ./scripts/install-rslidar-sdk.sh /scripts/
RUN dos2unix /scripts/install-rslidar-sdk.sh && chmod +x /scripts/install-rslidar-sdk.sh \
    && /bin/bash /scripts/install-rslidar-sdk.sh
RUN echo "source /home/$USERNAME/workspace/rslidar_build/install/setup.bash" >> ~/.bashrc

# copy scripts
COPY ./scripts/install-fastlio2-system.sh /scripts/
COPY ./scripts/install-fastlio2-ws.sh /scripts/
RUN dos2unix /scripts/*.sh && chmod +x /scripts/*.sh

# system part as root
USER root
RUN /bin/bash /scripts/install-fastlio2-system.sh

# workspace part as vscode
USER vscode
ENV TARGET_HOME=/home/vscode/workspace
RUN /bin/bash /scripts/install-fastlio2-ws.sh
RUN echo "source /home/vscode/workspace/fastlio2-build/install/setup.bash" >> ~/.bashrc

WORKDIR /home/$USERNAME/workspace
ENV LANG=en_US.UTF-8
ENV PYTHONWARNINGS="ignore"

ENTRYPOINT ["/bin/bash"]