# sick_wagen_docker
sick_wagen_workspaceをコンテナ内で動かす

## Docker commands
### Docker build
```
docker build -t sick/ros:noetic .
```
### Docker container implementation
```
rocker --x11 --nvidia --user --network=host --privileged --volume ./ --group-add dialout --volume /dev/tty* --volume /dev/video* --volume /dev/fdcanusb -- sick/ros:noetic
```
### Docker内での操作の例（stepbystepまで）
```
cd sick_wagen_ws/sick_wagen_workspace/sick_wagen/launch/stepbystep/
```

# SICK Wagen workspace

## Installation

```bash
sudo apt install ros-noetic-desktop-full \
    python3-catkin-tools \
    ros-noetic-serial \
    ros-noetic-ublox \
    ros-noetic-usb-cam \
    ros-noetic-mcl-3dl-msgs \
    ros-noetic-ecl-core \
    ros-noetic-move-base \
    ros-noetic-joy \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-dwa-local-planner \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-trajectory-tracker
```

```bash
mkdir ~/sick_wagen_ws
cd ~/sick_wagen_ws
git clone --recursive git@github.com:SICKxKM-TsukubaChallenge/sick_wagen_workspace.git
ln -sf sick_wagen_workspace src
cd src
git submodule foreach --recursive git checkout ros1-sick
cd ~/sick_wagen_ws
catkin build
```

## Devices

```bash
# Check vendor id, product id, and serial numbers using following commands
dmesg
# [ 4501.865120] cdc_acm 3-1.2.1:1.0: ttyACM0: USB ACM device
# [ 4569.094645] usb 3-1.2.1: USB disconnect, device number 30
# [ 4574.712856] usb 3-1.2.1: new full-speed USB device number 31 using xhci_hcd
# [ 4574.825699] usb 3-1.2.1: New USB device found, idVendor=1546, idProduct=01a9, bcdDevice= 1.00
# [ 4574.825703] usb 3-1.2.1: New USB device strings: Mfr=1, Product=2, SerialNumber=0
# [ 4574.825704] usb 3-1.2.1: Product: u-blox GNSS receiver
# [ 4574.825705] usb 3-1.2.1: Manufacturer: u-blox AG - www.u-blox.com
# [ 4574.834207] cdc_acm 3-1.2.1:1.0: ttyACM0: USB ACM device
lsusb
# Bus 004 Device 003: ID 0bda:8153 Realtek Semiconductor Corp. RTL8153 Gigabit Ethernet Adapter
# Bus 004 Device 002: ID 2109:0815 VIA Labs, Inc. USB3.0 Hub             
# Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 003 Device 003: ID 04f2:b760 Chicony Electronics Co., Ltd HP Wide Vision HD Camera
# Bus 003 Device 002: ID 03f0:6141 HP, Inc HP 280 Silent Wireless Mouse
# Bus 003 Device 004: ID 8087:0033 Intel Corp. 
# Bus 003 Device 019: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
# Bus 003 Device 025: ID 046d:c216 Logitech, Inc. F310 Gamepad [DirectInput Mode]
# Bus 003 Device 028: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
# Bus 003 Device 026: ID 2b72:0003 HP 280M HP 280 Silent Wireless Mouse
# Bus 003 Device 030: ID 1546:01a9 U-Blox AG USB2.0 Hub             
# Bus 003 Device 018: ID 0bda:5411 Realtek Semiconductor Corp. HP 280 Silent Wireless Mouse
# Bus 003 Device 024: ID 046d:082d Logitech, Inc. HD Pro Webcam C920
# Bus 003 Device 016: ID 2109:2815 VIA Labs, Inc. USB2.0 Hub             
# Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
# Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub# Create devices rules
sudo vim /etc/udev/rules.d/90-usb-serial-devices.rules
# Reload the udev service
sudo service udev reload
# Add user to dialout group
sudo adduser $USER dialout
```

/etc/udev/rules.d/90-usb-serial-devices.rules

```bash
# RT-USB-9AXIS
SUBSYSTEM=="tty", ATTRS{idVendor}=="2b72", ATTRS{idProduct}=="0003", SYMLINK+="ttyACM-RT9AXIS", GROUP="dialout"
# WT901C
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSB-WT901C", GROUP="dialout"
# WHILL
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTRVY2VA", SYMLINK+="ttyUSB-WHILL", GROUP="dialout"
# UBLOX
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyACM-UBLOX", GROUP="dialout"
```

## Network devices

```bash
sudo vim /etc/hosts
```

```bash
127.0.0.1       localhost
127.0.0.1       MY-PC
192.168.0.2     TIM_RIGHT_IP
192.168.0.3     TIM_LEFT_IP
192.168.0.4     MULTISCAN_IP
...
```

```bash
echo "export ROS_MASTER_PC_IP=192.168.0.152" >> ~/.bashrc
```

## URDF File

.bashrc
```
# SICK
export SICK_WAGEN_XACRO=sick_wagen.xacro
# KM
export SICK_WAGEN_XACRO=sick_wagen-km.xacro
```
