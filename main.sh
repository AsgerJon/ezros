#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#
clear
echo "main.sh"
echo "$HOME"
SCRIPT_DIR=$(dirname "$0")
export ROS_MASTER_URI=http://192.168.1.42:11311
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/src
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/src/ezros
export PYTHONPATH=$PYTHONPATH:$HOME/catkin_ws/devel/lib/python3.11
export PYTHONPATH=$PYTHONPATH:\
$HOME/catkin_ws/devel/lib/python3.11/site-packages
echo "$_CATKIN_SETUP_DIR"

cd "$HOME"/catkin_ws/ || exit
# rm -rf ./build
# rm -rf ./devel
# catkin_make
chmod +x "$HOME"/catkin_ws/devel/setup.zsh
echo "$HOME"/catkin_ws/devel/setup.zsh
source "$HOME"/catkin_ws/devel/setup.zsh
export ROS_PYTHON_ENV=/opt/miniconda3/envs/rosvist/bin/python3

source "$HOME"/.zshrc
cd "$HOME"/PycharmProjects/ezros/ || exit
"$ROS_PYTHON_ENV" main.py

