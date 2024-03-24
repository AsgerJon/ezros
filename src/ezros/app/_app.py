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

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
    self.setApplicationVersion('0.1.0')
    self.setOrganizationName('TMR')
    self.setOrganizationDomain('tmr.dk')
    self.setQuitOnLastWindowClosed(True)
    self.setWindowIcon(QIcon(':/icons/ezros.svg'))

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
