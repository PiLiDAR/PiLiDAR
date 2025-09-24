#!/bin/bash
#Author: David Hu
#Date: 2022-05

# Exit on error
set -e

if [ ! -e "./build" ];then
  mkdir build
  echo "create ./build/"
fi

echo "start cmake build"
cd ./build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j2