#!/usr/bin/env zsh
#
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
#

source /home/asgerjon/.zshrc

,mamba

mamba activate rosenv
export TALKERDIR=/home/asgerjon/PycharmProjects/ezros/src/ezros/talker
export EZROSDIR=/home/asgerjon/PycharmProjects/ezros/src
# export CATKINDIR=$HOME/catkin_ws/devel/lib/python3.11/site-packages
# export MSGSDIR=$HOME/catkin_ws/devel/lib/python3.11/site-packages/msgs
cd $TALKERDIR || exit

export PYTHONPATH=$PYTHONPATH:$TALKERDIR
export PYTHONPATH=$PYTHONPATH:$EZROSDIR
export PYTHONPATH=$PYTHONPATH:$CATKINDIR
export PYTHONPATH=$PYTHONPATH:$MSGSDIR

python _talk_run.py
