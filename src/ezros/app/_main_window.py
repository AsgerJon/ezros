"""The MainWindow class organizes the main application window."""
#  MIT Licence
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
from ezros.rosutils import RosThread, RollingArray, EZTimer

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  __zero_time__ = time.time()

  pumpCurrent = AttriBox[RosThread]('/tool/spray_current')
  pumpData = AttriBox[RollingArray](Settings.numPoints)
  pumpTimer = AttriBox[EZTimer](15, Precise)

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('EZROS')
    self.setMouseTracking(True)

  def initActions(self) -> None:
    """Initialize the actions."""
    ic('initActions')
    self.pumpCurrent.data.connect(self.receiveData)
    self.pumpTimer.timeout.connect(self.refreshChart)
    self.pumpCurrent.start()
    self.pumpTimer.start()

  def receiveData(self, data: complex) -> None:
    """Receive data from the ROS thread."""
    # self.pumpData.explicitAppend(data)
    data -= (self.__zero_time__ + 1e-08 * (random() - 0.5) * 1j)
    self.statusBar().showMessage('%.16E | %.16EI' % (data.real, data.imag))
    self.dynamicWidget.addData(data)

  def refreshChart(self, ) -> None:
    """Refresh the chart."""
    data = self.pumpData.complexNow()
    self.dynamicWidget.refreshChart()

  def closeEvent(self, event: QCloseEvent) -> None:
    """Close the window."""
    if not LayoutWindow.closeEvent(self, event):
      sys.exit(0)

  def debug1Func(self, ) -> None:
    """Debug function 1."""
    LayoutWindow.debug1Func(self)
    self.dynamicWidget.setMax()

  def debug2Func(self, ) -> None:
    """Debug function 2."""
    LayoutWindow.debug2Func(self)
    self.dynamicWidget.setMin()
