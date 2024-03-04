#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#
SCRIPT_DIR=$(dirname "$0")

export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/src
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/src/ezros
export PYTHONPATH=$PYTHONPATH:/home/AsgerJon/lmao
echo "$PYTHONPATH"

if [ -d /home/AsgerJon/catkin_ws/devel/setup.bash ]; then
  cd ~/catkin_ws/devel/ || exit
  source ./setup.bash
  cd /home/AsgerJon/PycharmProjects/ezros/ || exit
  export PYTHONPATH=$PYTHONPATH:\
  /home/asger/catkin_ws/devel/lib/python3.11/site-packages/
  export ROS_MASTER_URI=http://192.168.1.42:11311
  export ROS_IP=192.168.1.11
  export ROS_PYTHON_ENV=/opt/miniconda3/envs/rosvist/bin/python3
else
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_PYTHON_ENV=/opt/miniconda3/envs/rosvist/bin/python3
  export ROS_PYTHON_ENV=/home/AsgerJon/.conda/envs/rosenv/bin/python3
  echo "Catkin workspace not present, using yolo instead!"
fi

"$ROS_PYTHON_ENV" ./_pub_run.py