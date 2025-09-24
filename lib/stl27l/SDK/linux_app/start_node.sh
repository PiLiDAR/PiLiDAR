#!/bin/bash
#Author: David Hu
#Date: 2022-07

# Exit on error
set -e 

echo "Instructions:"
echo "  [product_type] = LD06/LD19/STL06P/STL27L/STL26"
echo "  [communication_mode] = serial/network"
echo "  [serial_port_number] = usb device mount files,example:/dev/ttyUSB0."
echo "  [server_ip] = TCP or UDP server host ip, example: 192.168.1.200"
echo "  [server_port] = TCP or UDP server host port, example: 2000"
echo "For Example:"
echo "  input [product_type]=LD06"
echo "  input [communication_mode]=serial"
echo "  input [serial_port_number]=/dev/ttyUSB0"
echo "  input [server_ip]=<Enter key>"
echo "  input [server_port]=<Enter key>"
echo " or "
echo "  input [product_type]=LD06"
echo "  input [communication_mode]=network"
echo "  input [serial_port_number]=<Enter key>"
echo "  input [server_ip]=192.168.1.200"
echo "  input [server_port]=2000"

read -p "input [product_type]=" PRODUCT_TYPE
read -p "input [communication_mode]=" COMM_MODE
read -p "input [serial_port_number]=" PORT_PATH
read -p "input [server_ip]=" SERVER_IP_ADDR
read -p "input [server_port]=" SERVER_PORT_ADDR

if [ ! ${PRODUCT_TYPE} ]
then
  echo "ERROR [product_type] input is null"
  exit 0
fi

if [ ! ${COMM_MODE} ]
then
  echo "ERROR [communication_mode] input is null"
  exit 0
fi

LOG_NAME="test`date +%Y%m%d-%H-%M`.log"
if [ $COMM_MODE == "network" ]
then
  echo "start node exec"
  echo "output log to ./${LOG_NAME}"
  if [ ! -e "./log" ]
  then
    mkdir log
    echo "create ./log/"
  fi
  ./build/ldlidar_stl ${PRODUCT_TYPE} networkcom_tcpclient ${SERVER_IP_ADDR} ${SERVER_PORT_ADDR} > ./log/${LOG_NAME} 
elif [ $COMM_MODE == "serial" ]
then
  sudo chmod 777 ${PORT_PATH}
  echo "start node exec"
  echo "output log to ./${LOG_NAME}"
  if [ ! -e "./log" ]
  then
    mkdir log
    echo "create ./log/"
  fi
  ./build/ldlidar_stl ${PRODUCT_TYPE} serialcom ${PORT_PATH} > ./log/${LOG_NAME} 
else
  echo "input [communication_mode] is error."
fi
