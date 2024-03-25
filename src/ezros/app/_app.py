"""App subclasses the QApplication class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import json
import os

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication


class App(QApplication):
  """App is a subclass of QApplication."""

  icons = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')

  def getSettingsFile(self) -> str:
    """Returns the path to the defaults file"""
    appFolder = self.applicationDirPath()
    return os.path.join(appFolder, 'defaults.json')

  def loadSettings(self) -> dict:
    """Loads the defaults from the defaults file."""
    settingsFile = self.getSettingsFile()
    if os.path.exists(settingsFile):
      with open(settingsFile, 'r') as file:
        return json.load(file)
    return {}

  def saveSettings(self, settings: dict) -> None:
    """Saves the defaults to the defaults file."""
    settingsFile = self.getSettingsFile()
    with open(settingsFile, 'w') as file:
      json.dump(settings, file, indent=2)

  def _getIconPath(self) -> str:
    """Getter-function for the icon path."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'icons')

  def _loadIcons(self) -> None:
    """Loads the icons for the application."""
    iconPath = self._getIconPath()
    for item in os.listdir(iconPath):
      if item.endswith('.png'):
        icon = QIcon(os.path.join(iconPath, item))
        self.setWindowIcon(icon)
        break
