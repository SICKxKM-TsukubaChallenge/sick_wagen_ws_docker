# SICK Wagen workspace

## Installation

```bash
sudo apt install ros-noetic-serial ros-noetic-ublox

```

```bash
mkdir ~/catkin_ws
cd catkin_ws
git clone --recursive https://github.com/SICKxKM-TsukubaChallenge/sick_wagen_workspace
ln -sf sick_wagen_workspace src
catkin build
```
