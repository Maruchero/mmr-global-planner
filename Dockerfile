FROM ubuntu:focal-20240410 

# add new user
RUN useradd -ms /bin/bash mmr -G sudo

# install tool
RUN apt update && \
	apt install -y software-properties-common && \
	add-apt-repository universe && \
	apt update && \
	apt install -y curl && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
	apt update && \
	apt install -y build-essential \
	ros-foxy-desktop \
	python3-argcomplete \
	ros-dev-tools \
	ros-foxy-rosbag2-storage-mcap \
  	ros-foxy-ackermann-msgs

RUN mkdir /var/run/sshd
RUN echo "mmr:mmr" | chpasswd

COPY --chown=mmr:mmr ./bashrc /home/mmr/.bashrc
RUN mkdir -p /home/mmr/.cache/xdgr && chown mmr:mmr /home/mmr/.cache/xdgr
RUN apt install -y python3-pip

COPY ./gp_env/src/mmrGlobalPlanner/requirements.txt /home/mmr/requirements.txt

# USER mmr

RUN python3 -m pip uninstall numpy scipy matplotlib quadprog trajectory_planning_helpers pyyaml
RUN python3 -m pip install -r /home/mmr/requirements.txt

USER mmr
WORKDIR /home/mmr/gp_env

# start sshd
CMD ["/bin/bash"]