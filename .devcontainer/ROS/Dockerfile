# Install Python 3.10
RUN apt-get update && \
    apt-get upgrade && \
    apt-get install ros-humble-ackermann-msgs
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.10 python3.10-venv python3.10-dev && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# Automatically source ROS setup in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc