# SICK Wagen workspace

## Installation

```bash
sudo apt install ros-noetic-serial ros-noetic-ublox
```

```bash
mkdir ~/sick_wagen_ws
cd sick_wagen_ws
git clone --recursive https://github.com/SICKxKM-TsukubaChallenge/sick_wagen_workspace
git submodule foreach --recursive git checkout ros1-sick
ln -sf sick_wagen_workspace src
catkin build
```
