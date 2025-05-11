# mmr-global-planner

Dockerized version of the 2025 UNIMORE Formula Student Driverless ROS package for race line optimization, originally created by [jacksalici](https://github.com/jacksalici/mmrGlobalPlanner).

## Table of contents

- [1. Requirements](#1-requirements)
- [2. Running](#2-running)

## 1. Requirements

- Docker

## 2. Running

To run the container you first need the image. Currently the only way to get it is to build it yourself.

```bash
docker build . -t mmr-global-planner
```

Then you can run the container, mount a volume and connect interactively to its shell.

```bash
docker run --name mmr-global-planner -v .:/mmr -it mmr-global-planner
```

**Notice** that "`-v .:/mmr`" mounts a volume from the current directory to `/mmr`. Make sure this matches your needs.

After that you should build the packages by running:

```bash
colcon build --symlink-install
```

After that you can source your environment.

```bash
source install/setup.bash
```

At this point you will be able to launch the global planner node.

```bash
ros2 launch global_planner global_planner.launch.py
```
