"""The MainWindow class organizes the main application window."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from random import random
import sys
import time

from PySide6.QtGui import QCloseEvent
from attribox import AttriBox
from ezside.core import Precise
from icecream import ic

from ezros.app import LayoutWindow
from ezros.defaults import Settings
from ezros.rosutils import SubRos, RollingArray, EZTimer

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def __init__(self, ) -> None:
    """Initialize the main window."""
    LayoutWindow.__init__(self, )
    self.setWindowTitle('EZROS')
    self.rosThread = SubRos('/tool/pump_current')

  def initActions(self) -> None:
    """Initialize the actions."""
    self.rosThread.data.connect(self.onData)
    self.rosThread.start()

  def closeEvent(self, event: QCloseEvent) -> None:
    """Close the window."""
    if not LayoutWindow.closeEvent(self, event):
      sys.exit(0)

  def onData(self, data: complex) -> None:
    """Update the data."""
    value = data.imag
    self.baseLabel.setText('Pump Current: %.6E' % value)
