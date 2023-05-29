#!/bin/bash

# designed to run in /home directory and creates the following directories:

# /home
#   |- /fst
#       |- /simulator
#       |- /software10d

mkdir fst && cd fst

source /opt/ros/melodic/setup.bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install --no-install-recommends -y python-catkin-tools git python-rosdep ros-melodic-velodyne libcgal-dev libcgal-qt5-dev python3-pip python3-dev python3-numpy ros-melodic-libg2o libsuitesparse-dev

mkdir -p catkin_build_ws/src  && cd catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
cd src && git clone -b melodic https://github.com/ros-perception/vision_opencv.git 
cd ../
catkin build cv_bridge
echo "source ~/fst/catkin_build_ws/install/setup.bash --extend" >> ~/.bashrc

cd ../
mkdir -p simulator/src && cd simulator
catkin init
cd src && git clone git@gitlab.com:projectofst/fssim.git
cd fssim
y | ./update_dependencies.sh
#catkin config --extend /opt/ros/melodic
catkin build
echo "source ~/fst/simulator/devel/setup.bash" >> ~/.bashrc

pip3 install scikit-build
pip3 install rospkg torch torchvision numpy opencv-python

cd ../../../
git clone git@gitlab.com:projectofst/software10d.git
cd software10d/autonomous-system/src/estimation/graphslam/missing_libg2o_files
sudo cp FindCSparse.cmake  /opt/ros/melodic/share/cmake_modules/cmake/Modules/
sudo cp -r cmake/ /opt/ros/melodic/share/libg2o/
cd ../../../../
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
echo "source ~/fst/software10d/autonomous-system/fst_environment.sh" >> ~/.bashrc
./fst_environment.sh
sudo chmod -R 777 /opt/ros/melodic/share/libg2o/cmake/
sudo chmod -R 777 /opt/ros/melodic/share/cmake_modules/cmake/Modules/
source ~/.bashrc
catkin_make -DCATKIN_BLACKLIST_PACKAGES="arena_camera;ouster_ros" -DOpenCV_DIR=/usr/share/OpenCV/
