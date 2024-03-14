"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from math import pi, sin

from PySide6.QtCore import QPointF
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QGraphicsView
from icecream import ic
from vistside.core import TimerField, Precise
from vistside.windows import BaseWindow

from tester_class_01 import TestField

ic.configureOutput(includeContext=True)


class TestWindow(BaseWindow):
  """The TestWindow class is a window that contains a layout of widgets."""

  testWidget = TestField()
  testTimer = TimerField(100, Precise, singleShot=False)

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestWindow."""
    BaseWindow.__init__(self, *args, **kwargs)
    self.setMinimumSize(800, 600)

  def show(self) -> None:
    """Show the window."""
    self.testWidget.initUi()
    self.baseLayout.addWidget(self.testWidget)
    self.testWidget.mouse.connect(self.debugMouse)
    self.testTimer.timeout.connect(self.showValue)
    BaseWindow.show(self)
    self.testTimer.start()

  def debug01Func(self, ) -> None:
    """Debug function 01"""
    print('debug01Func')
    self.testWidget.update()
    self.statusBar().showMessage('debug01Func')

  def debugMouse(self, x: float, y: float) -> None:
    """Debug mouse"""
    Q = self.testWidget.viewField.chart.mapToValue(QPointF(x, y))
    msg = 'x: %03d: %.3f | y: %03d: %.3f' % (x, Q.x(), y, Q.y())
    self.statusBar().showMessage(msg)

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Mouse release event"""
    ic(self.testWidget.viewField.scene())

  def showValue(self, ) -> None:
    """Show value"""
    t = time.time() % (2 * pi)
    x = sin(t)
    self.testWidget.viewField.chart.series().append(QPointF(t, x))
