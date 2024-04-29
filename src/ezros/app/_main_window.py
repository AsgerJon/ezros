"""The MainWindow class organizes the main application window."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys

from PySide6.QtCore import Signal
from PySide6.QtGui import QCloseEvent
from icecream import ic
from msgs.msg import Float32Stamped
from rospy import signal_shutdown

from ezros.app import LayoutWindow
from ezros.rosutils import SubRos, BoolPub

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  acceptQuit = Signal()

  def __init__(self, ) -> None:
    """Initialize the main window."""
    LayoutWindow.__init__(self, )
    self.setWindowTitle('EZROS')
    self.pumpCurrentThread = SubRos('/tool/pump_current')
    self.pumpCommandThread = BoolPub('/tool/pump_command')
    self.sprayCommandThread = BoolPub('/tool/spray_command')

  def initActions(self) -> None:
    """Initialize the actions."""
    self.pumpCurrentThread.data.connect(self.onData)
    self.pumpControl.activated.connect(self.pumpCommandThread.activate)
    self.pumpControl.deactivated.connect(self.pumpCommandThread.deactivate)
    self.sprayControl.activated.connect(self.sprayCommandThread.activate)
    self.sprayControl.deactivated.connect(self.sprayCommandThread.deactivate)
    # self.requestQuit.connect(self.pumpCurrentThread.stop)
    # self.requestQuit.connect(self.pumpCommandThread.stop)
    # self.requestQuit.connect(self.sprayCommandThread.stop)
    self.requestQuit.connect(self.close)
    self.pumpCurrentThread.start()
    self.pumpCommandThread.start()
    self.sprayCommandThread.start()

  def onData(self, data: Float32Stamped) -> None:
    """Update the data."""
    value = data.data
    self.pumpCurrentLabel.setText('Pump Current: %.12E' % value)
    self.pumpData.append(data)

  def debug2Func(self, ) -> None:
    """Debug2 function."""
    LayoutWindow.debug2Func(self)

  def closeEvent(self, event: QCloseEvent) -> None:
    """Close event."""
    self.pumpCurrentThread.stop()
    self.pumpCommandThread.stop()
    self.sprayCommandThread.stop()
    LayoutWindow.closeEvent(self, event)
