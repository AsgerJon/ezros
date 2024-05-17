"""The 'rosutils' module provides standalone functionalities for accessing
ROS through Python. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._ros_topic import RosTopic
from ._directories import CATKIN_WS, baseCMakeLists, basePackageXml
from ._directories import basePackageXmlFile, baseCMakeListsFile, ROS_UTILS
from ._msg_types import getMsgTypes, getMsgTypeNames, resolveMsgType
from ._generate_ros_package import generateRosPackage, getRosPackageDir
from ._catkin_make import runCatkinMake
from ._update_ros_packages import updateRosPackages

if __name__ != '__main__':
  updateRosPackages()
