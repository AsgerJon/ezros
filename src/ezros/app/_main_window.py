"""The MainWindow class organizes the main application window."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys

from PySide6.QtGui import QCloseEvent
from icecream import ic
from msgs.msg import Float32Stamped
from rospy import Time

from ezros.app import LayoutWindow
from ezros.rosutils import SubRos, BoolPubRos

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def __init__(self, ) -> None:
    """Initialize the main window."""
    LayoutWindow.__init__(self, )
    self.setWindowTitle('EZROS')
    self.pumpCurrentThread = SubRos('/tool/pump_current')
    self.pumpCommandThread = BoolPubRos('/tool/pump_command')

  def initActions(self) -> None:
    """Initialize the actions."""
    self.pumpCurrentThread.data.connect(self.onData)
    self.pumpCurrentThread.start()
    # self.pumpCommandThread.start()
    # self.activatePump.clicked.connect(self.pumpCommandThread.activate)
    # self.deactivatePump.clicked.connect(self.pumpCommandThread.deactivate)
    # self.pumpCommandThread.stateChanged.connect(self.handleStateChange)

  def closeEvent(self, event: QCloseEvent) -> None:
    """Close the window."""
    if not LayoutWindow.closeEvent(self, event):
      sys.exit(0)

  def onData(self, data: Float32Stamped) -> None:
    """Update the data."""
    value = data.data
    self.baseLabel.setText('Pump Current: %.12E' % value)
    self.pumpData.append(data)

  #
  # def handleStateChange(self, state: bool) -> None:
  #   """Handle the state change."""
  #   self.pumpStatus.setText('Pump Status: %s' % ('ON' if state else 'OFF'))

  def debug2Func(self, ) -> None:
    """Debug2 function."""
    LayoutWindow.debug2Func(self)
