"""App subclasses the QApplication class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from icecream import ic
from rospy import init_node
from vistutils.text import monoSpace

from ezros.rosutils import sourceCatkin, initNodeMaybe

ic.configureOutput(includeContext=True)
MenuFlag = Qt.ApplicationAttribute.AA_DontUseNativeMenuBar


class App(QApplication):
  """App is a subclass of QApplication."""

  __caller_id__ = None
  __catkin_source__ = sourceCatkin()

  icons = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
    self.setAttribute(MenuFlag, True)
    self.__caller_id__ = initNodeMaybe('EZROS', anonymous=False)
