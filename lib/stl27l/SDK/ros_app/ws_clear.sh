#!/bin/bash
#Author: David Hu
#Date: 2020-12
rm ./.catkin_workspace
echo "del .catkin_workspace file"
rm -rf ./build
echo "del build floder"
rm -rf ./devel
echo "del devel floder"
rm -rf ./src/CMakeLists.txt
echo "del src/CMakeLists.txt file"
echo "del is ok....."


