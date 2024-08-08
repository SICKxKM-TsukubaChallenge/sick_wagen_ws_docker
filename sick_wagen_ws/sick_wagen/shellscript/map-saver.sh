#!/bin/bash

now=$(date +"%Y-%m-%d-%H%M%S")

dir=~/sick_wagen_ws/maps
if [ ! -d "${dir}" ]; then
	echo mkdir -p ${dir}
	mkdir -p "${dir}"
fi

filename="map_${now}"
echo rosrun map_server map_saver -f $dir/$filename
if ! rosrun map_server map_saver -f $dir/$filename; then
	echo "ERROR: Failed to save file $filename"
	exit 1
fi

echo If you want to use this map, enter the following command.
echo export ROS_MAP=$dir/$filename
