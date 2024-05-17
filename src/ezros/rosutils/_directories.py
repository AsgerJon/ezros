"""This file allows the module to import the names of the base files. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.env import getParentDir, changeDir

ROS_UTILS = getParentDir(__file__)
CATKIN_WS = changeDir(ROS_UTILS, 'catkin_ws')
baseCMakeListsFile = changeDir(ROS_UTILS, '_base_cmakelists_txt.txt')
basePackageXmlFile = changeDir(ROS_UTILS, '_base_package_xml.txt')

with open(baseCMakeListsFile, 'r') as f:
  baseCMakeLists = f.read()

with open(basePackageXmlFile, 'r') as f:
  basePackageXml = f.read()
