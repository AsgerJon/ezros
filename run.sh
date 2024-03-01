#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#
export PYTHONPATH=$PYTHONPATH:./src
export PYTHONPATH=$PYTHONPATH:./src/ezros
export PYTHONPATH=$PYTHONPATH:\
/home/asger/catkin_ws/devel/lib/python3.11/site-packages/

cd ~/catkin_ws/devel/ || exit
source ./setup.bash
cd ~/PycharmProjects/ezros/ || exit

export ROS_MASTER_URI=http://localhost:11311
# export ROS_MASTER_URI=http://192.168.1.42:11311
export ROS_IP=192.168.1.11

/opt/miniconda3/envs/rosvist/bin/python3 run.py
