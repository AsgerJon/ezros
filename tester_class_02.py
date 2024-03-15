"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from math import pi, sin

from PySide6.QtCore import QPointF, Slot
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QGraphicsView
from icecream import ic
from vistside.core import TimerField, Precise
from vistside.windows import BaseWindow

from ezros.gui import PlotWidget, PlotField, DataChart
from tester_class_01 import TestField

ic.configureOutput(includeContext=True)


class TestWindow(BaseWindow):
  """The TestWindow class is a window that contains a layout of widgets."""

  testTimer = TimerField(100, Precise, singleShot=False)

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestWindow."""
    BaseWindow.__init__(self, *args, **kwargs)
    self.setMinimumSize(800, 600)
    self.testTimer.timeout.connect(self.addValue)
    self.plotWidget = None

  def show(self) -> None:
    """Show the window."""
    self.plotWidget = DataChart()
    self.plotWidget.initUi()
    self.baseLayout.addWidget(self.plotWidget)
    BaseWindow.show(self)
    self.testTimer.start()

  def debug01Func(self, ) -> None:
    """Debug function 01"""
    print('debug01Func')
    self.plotWidget.update()
    self.statusBar().showMessage('debug01Func')

  @Slot()
  def addValue(self, ) -> None:
    """Append a value to the data array."""
    value = sin(time.time())
    self.plotWidget.appendValue(value)
    self.plotWidget.update()
    self.statusBar().showMessage('Value: %.3f' % value)
