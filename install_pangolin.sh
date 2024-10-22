#!/bin/bash
workspace="$USER/hang_ws"
project_name="my_slam_auto_driving"
cd $workspace/$project_name/thirdpary
unzip Pangolin.zip
cd Pangolin
cmake -B build 
cmake --build build
