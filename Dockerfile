FROM ros:foxy

WORKDIR /mmr

COPY . .

RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y ros-${ROS_DISTRO}-ackermann-msgs python3-pip && \
  pip install -r src/mmrGlobalPlanner/requirements.txt && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
