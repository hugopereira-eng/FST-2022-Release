#!/bin/bash

RED="\033[0;31m"
GREEN="\033[0;32m"
NC="\033[0m"
FLAG=$1
VCAN0=$2
VCAN1=$3
if [ $# -eq 0 ]
then
	echo -e "Please insert c or d flag to create or delete the vcan lines"
	exit 1

elif [ $# -eq 1 ]
then
	if [ $FLAG == 'c' ]
	then
		sudo modprobe vcan
		sudo ip link add dev vcan0 type vcan 2>/dev/null
		if [ $? -ne 0 ]
		then
			sudo ip link del vcan0
			sudo ip link add dev vcan0 type vcan
		fi
		sudo ip link add dev vcan1 type vcan 2>/dev/null
		if [ $? -ne 0 ]
		then
			sudo ip link del vcan1
			sudo ip link add dev vcan1 type vcan
		fi

		sudo ip link set up vcan0
		sudo ip link set up vcan1
		echo -e "${GREEN}Virtual CAN BUS lines setup successfully.${NC}"
	elif [ $FLAG == 'd' ]
	then
		sudo ip link del vcan0
		sudo ip link del vcan1
		echo -e "${GREEN}Successfully deleted vcan0 and vcan1${NC}"
	else
		echo -e "${RED}Invalid flags please use ${GREEN}'c'${RED} or ${GREEN}'d'${RED} flags${NC}"
	fi
fi
