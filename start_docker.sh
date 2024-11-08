#!/bin/bash
rocker --x11 --nvidia --user --network=host --privileged --volume ./ --group-add dialout --volume /dev/tty* --volume /dev/video* --volume /dev/fdcanusb -- sick/ros:noetic
