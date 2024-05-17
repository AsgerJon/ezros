"""The 'generateRosPackage' creates a basic entry in the catkin_ws/src
directory."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from ezros.env import changeDir
from ezros.rosutils import CATKIN_WS, basePackageXml, baseCMakeLists


def getRosPackageDir(packageName: str) -> str:
  """Returns the path to the ROS package directory."""
  return changeDir(CATKIN_WS, 'src', packageName)


def _createDirectory(packageName: str) -> None:
  """Creates the catkin_ws/src directory if it does not exist."""
  packageDir = getRosPackageDir(packageName)
  if os.path.exists(packageDir):
    return
  os.makedirs(packageDir)
  msgDir = changeDir(packageDir, 'msg')
  os.makedirs(msgDir)


def _generatePackageXML(packageName: str) -> None:
  """Generates the package.xml file for the ROS package."""
  newFilePath = changeDir(getRosPackageDir(packageName), 'package.xml')
  newContent = basePackageXml.replace('{{PACKAGE_NAME}}', packageName)
  with open(newFilePath, 'w') as f:
    f.write(newContent)


def _generateCMakeLists(rosPackage: str) -> None:
  """Updates the CMakeLists.txt in the given rosPackage directory based on
  the msg files present."""
  newFile = baseCMakeLists.replace('{{PACKAGE_NAME}}', rosPackage)
  rosPackageDir = getRosPackageDir(rosPackage)
  rosMsgDir = changeDir(rosPackageDir, 'msg')
  msgFiles = []
  for item in os.listdir(rosMsgDir):
    if item.endswith('.msg'):
      msgFiles.append(item)
  msgFileEntry = '  \n'.join(msgFiles)
  newFile = newFile.replace('{{MSG_FILES}}', msgFileEntry)
  newFilePath = os.path.join(rosPackageDir, 'CMakeLists.txt')
  with open(newFilePath, 'w') as f:
    f.write(newFile)


def generateRosPackage(packageName: str) -> None:
  """Generates a basic ROS package in the catkin_ws/src directory."""
  _createDirectory(packageName)
  _generateCMakeLists(packageName)
  _generatePackageXML(packageName)
