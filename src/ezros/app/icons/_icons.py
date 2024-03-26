"""Icons is a class providing PIL images for each icon"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PIL import Image
from PySide6.QtGui import QIcon


class Icons:
  """Icons is a class providing PIL images for each icon"""

  @staticmethod
  def _getIconsPath() -> str:
    """Getter-function for the icons path"""
    return os.path.dirname(os.path.abspath(__file__))

  @staticmethod
  def _loadImg(iconPath: str) -> Image:
    """Loads an icon from the given path"""
    return Image.open(iconPath)

  def __init__(self, ) -> None:
    self._icons = dict()
    for item in os.listdir(self._getIconsPath()):
      if item.endswith('.png'):
        name = item.split('.')[0]
        path = os.path.join(self._getIconsPath(), item)
        self._addIcon(name, path)

  def _addIcon(self, name: str, path: str) -> None:
    """Adds an icon to the icons dictionary"""
    self._icons[name] = path

  def getImg(self, name: str) -> Image:
    """Returns the icon with the given name"""
    filePath = self._icons[name]
    return self._loadImg(filePath)

  def getIcon(self, name: str) -> QIcon:
    """Returns the icon with the given name"""
    return QIcon(self._icons[name])
