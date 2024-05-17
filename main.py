"""Main Tester Script"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
import time
from typing import Callable

from PySide6.QtCore import Qt
from icecream import ic
import roslib

from ezros.app import EZRos, MainWindow
from ezros.env import SITE_PACKAGES, EZ_ROOT
import std_msgs.msg as baseMsg
import yolo.msg as yolo
from ezros.rosutils import getMsgTypeNames, getMsgTypes

ic.configureOutput(includeContext=True, )


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, ]
  for item in stuff:
    print(item)


def tester01() -> int:
  """Main application tester"""
  return EZRos(MainWindow).exec()


def tester02() -> int:
  """Testing Ros stuff"""
  for item in dir(roslib.packages):
    try:
      print(item)
    except Exception as exception:
      print(exception)
      return 1
  ic(roslib.packages.catkin_find())
  ic(roslib.packages.rospkg)
  return 0


def tester03() -> int:
  """Testing env module"""
  try:
    ic(SITE_PACKAGES)
    ic(EZ_ROOT)
  except Exception as exception:
    print(exception)
    return 1
  return 0


def tester04() -> int:
  """Testing std_msgs.msg"""
  ic(os.environ['CONDA_PREFIX'])
  ic(os.path.basename(os.environ['CONDA_PREFIX']))
  for (name, cls) in getMsgTypes().items():
    try:
      print(name)
      print(cls.__bases__)
      break
    except Exception as exception:
      print(exception)
      return 1
  else:
    return 0
  return 0


def tester05() -> int:
  """Testing yolo"""
  try:
    ic(yolo)
  except Exception as exception:
    print(exception)
    return 1
  return 0


def main(callMeMaybe: Callable) -> None:
  """Main Tester Script"""
  tic = time.time()
  print('Running python script located at: \n%s' % sys.argv[0])
  print('Started at: %s' % time.ctime())
  print(77 * '-')
  retCode = 0
  try:
    retCode = callMeMaybe()
  except Exception as exception:
    print('Exception: %s' % exception)
    raise exception
  retCode = 0 if retCode is None else retCode
  print(77 * '-')
  print('Return Code: %s' % retCode)
  print('Runtime: %.3f seconds' % (time.time() - tic))


if __name__ == '__main__':
  main(tester01)
