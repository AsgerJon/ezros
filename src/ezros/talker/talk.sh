#!/usr/bin/env zsh
#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#

source /home/AsgerJon/.zshrc

,mamba

mamba activate rosenv
export TALKERDIR=/home/AsgerJon/PycharmProjects/ezros/src/ezros/talker
export EZROSDIR=/home/AsgerJon/PycharmProjects/ezros/src
export CATKINDIR=$HOME/catkin_ws/devel/lib/python3.11/site-packages
export MSGSDIR=$HOME/catkin_ws/devel/lib/python3.11/site-packages/msgs
cd $TALKERDIR || exit

export PYTHONPATH=$PYTHONPATH:$TALKERDIR
export PYTHONPATH=$PYTHONPATH:$EZROSDIR
export PYTHONPATH=$PYTHONPATH:$CATKINDIR
export PYTHONPATH=$PYTHONPATH:$MSGSDIR

python _talk_run.py