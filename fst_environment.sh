#!/bin/bash

CATKIN_SHELL=bash

export FST_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)

source ${FST_ROOT}/fst_aliases

# check whether devel folder exists
if [ -f "${FST_ROOT}/devel/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "${FST_ROOT}/devel/setup.bash"
else
    source "/opt/ros/melodic/setup.bash"
    printf "Run 'asmake' in the autonomous-system directory to compile\n"
fi
