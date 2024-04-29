#!/usr/bin/env zsh
#
# GPL-3.0 license
# Copyright (c) 2024 Asger Jon Vistisen
#



# >>> conda initialize >>>

export QT_PLUGIN_PATH="/usr/lib/qt6/plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/qt6/plugins/platforms"


function ,mamba() {
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/AsgerJon/miniforge3/bin/conda' 'shell.zsh' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/AsgerJon/miniforge3/etc/profile.d/conda.sh" ]; then
        . "/home/AsgerJon/miniforge3/etc/profile.d/conda.sh"
    else
        export PATH="/home/AsgerJon/miniforge3/bin:$PATH"
    fi
fi
unset __conda_setup

if [ -f "/home/AsgerJon/miniforge3/etc/profile.d/mamba.sh" ]; then
    . "/home/AsgerJon/miniforge3/etc/profile.d/mamba.sh"
fi

# <<< conda initialize <<<
}

function ,ros() {
  ,mamba
  mamba activate rosenv
  export ROS_MASTER_URI="http://192.168.1.83:11311"
  export ROS_IP="192.168.1.165"
  export MAMBA_ROS="/home/AsgerJon/miniforge3/envs/ros311"
  export CATKIN_DIR="/home/AsgerJon/catkin_ws"
  source "$CATKIN_DIR/devel/setup.zsh"
  export PYTHONPATH="$PYTHONPATH:$CATKIN_DIR/devel/lib/python3.11/site-packages/"
}

,ros

export PYTHONPATH="$PYTHONPATH:/home/AsgerJon/PycharmProjects/ezros/src"
python /home/AsgerJon/PycharmProjects/ezros/main.py