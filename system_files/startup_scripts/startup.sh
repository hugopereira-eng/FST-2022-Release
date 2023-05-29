#!/bin/bash

dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eno1 --bind-dynamic
#dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eno1 --bind-dynamic &
#ptp4l -i enp6s0 -m &
#ptp4l -i eno1 -m 

