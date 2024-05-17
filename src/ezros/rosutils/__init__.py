"""The 'rosutils' module provides standalone functionalities for accessing
ROS through Python. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from ._ros_topic import RosTopic
from ._directories import CATKIN_WS, baseCMakeLists, basePackageXml
from ._directories import basePackageXmlFile, baseCMakeListsFile, ROS_UTILS
from ._msg_types import getMsgTypes, getMsgTypeNames, resolveMsgType
from ._generate_ros_package import generateRosPackage, \
  getRosPackageDir, \
  getRosPackages
from ._catkin_make import runCatkinMake, _realPythonPath
from ._update_ros_packages import updateRosPackages

ic.configureOutput(includeContext=True)

if __name__ != '__main__' and False:
  for item in getRosPackages():
    ic(item)
  ic(_realPythonPath())

  updateRosPackages()
