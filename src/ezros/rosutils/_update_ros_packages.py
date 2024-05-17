"""The 'updateRosPackages' ensures that all modules present in the catkin
workspace is accounted for in the maker files. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from ezros.env import changeDir
from ezros.rosutils import CATKIN_WS, generateRosPackage, runCatkinMake


def updateRosPackages() -> None:
  """updateRosPackages ensures that all modules present in the catkin
  workspace is accounted for in the maker files. """
  src = changeDir(CATKIN_WS, 'src')
  for item in os.listdir(src):
    packDir = changeDir(src, item)
    if os.path.isfile(packDir):
      continue
    if item.startswith('.'):
      continue
    generateRosPackage(item)
  runCatkinMake()
