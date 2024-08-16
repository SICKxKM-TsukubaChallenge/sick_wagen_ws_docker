#!/bin/bash

# Define the common options
VOLUME_OPTS_MINIMUM="--volume $(pwd):/root/sick_wagen_ws_docker"
VOLUME_OPTS="--volume $(pwd):/root/sick/sick_wagen_ws_docker --group-add dialout --volume /dev/tty* --volume /dev/video* --volume /dev/fdcanusb"
IMAGE_NAME="sick/ros:noetic"
DOCKER_CMD="docker run --rm -it"
ROCKER_CMD="rocker --x11 --nvidia --user --network=host --privileged $VOLUME_OPTS $IMAGE_NAME"

# Check if -d option is passed
if [ "$1" == "-d" ]; then
    shift # Remove the -d option from the argument list
    CMD="$DOCKER_CMD $VOLUME_OPTS_MINIMUM $@ $IMAGE_NAME"
else
    CMD="$ROCKER_CMD $@"
fi

# Execute the command
echo "Executing: $CMD"
eval $CMD