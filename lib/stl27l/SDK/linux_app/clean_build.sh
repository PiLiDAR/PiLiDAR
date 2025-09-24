#!/bin/bash
#Author: David Hu
#Date: 2022-10
if [ -e "./build" ];then
  rm -rf build/
  echo "del ./build/"
fi

if [ -e "./log" ];then
  rm -rf log/
  echo "del ./log/"
fi

echo "del is ok....."