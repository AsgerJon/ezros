#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#
export PYTHONPATH=$PYTHONPATH:./src
export PYTHONPATH=$PYTHONPATH:./src/ezros
export PYTHONPATH=$PYTHONPATH:\
/home/asger/catkin_ws/devel/lib/python3.11/site-packages/


cd ~/catkin_ws/devel/ || exit
ls -latr
source ./setup.bash
cd ~/PycharmProjects/ezros/ || exit

/opt/miniconda3/envs/rosvist/bin/python3 main.py
