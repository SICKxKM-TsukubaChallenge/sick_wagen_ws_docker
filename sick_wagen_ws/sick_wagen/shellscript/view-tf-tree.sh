#!/bin/bash

DIR="/home/sick/sick_wagen_ws/tf_tree"
mkdir -p $DIR

cd $DIR
rosrun tf view_frames

xdot frames.gv &
