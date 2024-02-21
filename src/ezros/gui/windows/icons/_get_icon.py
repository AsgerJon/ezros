"""Getter-function for the icon of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtGui import QIcon, QPixmap
from vistutils.text import monoSpace


def _getPath(actionName: str) -> str:
  """Returns the path to the icon of the given action."""
  actionName = actionName.replace('.', '').replace(' ', '')
  if 'save' in actionName and 'as' in actionName:
    actionName = 'save_as'
  here = os.path.dirname(os.path.abspath(__file__))
  iconPath = os.path.join(here, actionName.lower() + '.png')
  if os.path.exists(iconPath):
    if os.path.isfile(iconPath):
      return iconPath
    e = """Expected a file, but found a directory at the given path: '%s'!"""
    raise NotADirectoryError(monoSpace(e % iconPath))
  e = """No file found at the given path: '%s'!"""
  raise FileNotFoundError(monoSpace(e % iconPath))


def _handle(actionName: str) -> str:
  """Returns the path to the icon of the given action."""
  try:
    return _getPath(actionName)
  except FileNotFoundError as fileNotFoundError1:
    try:
      return getIcon('risitas')
    except Exception as exception:
      raise exception from fileNotFoundError1


def getIcon(actionName: str) -> QIcon:
  """Returns the icon of the given action."""
  return QIcon(_handle(actionName.strip().lower()))


def getPixmap(actionName: str) -> QIcon:
  """Returns the icon of the given action."""
  return QPixmap(_handle(actionName.strip().lower()))
