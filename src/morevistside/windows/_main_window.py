"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

import numpy as np
import rospy
from PySide6.QtCore import QTimer, Qt
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from morevistside.windows import LayoutWindow


class MainWindow(LayoutWindow):
  """MainWindow subclasses LayoutWindow and provides the business logic."""

  __paint_timer__ = None
  paintTimer = Field()

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self._subscriber = None

  def receiveData(self, data: Any = None) -> None:
    """Test function"""
    if data is None:
      return self.timePlot.receiveValues(time.time(), np.nan)
    try:
      return self.timePlot.receiveValues(time.time(), data.data)
    except AttributeError:
      return self.timePlot.receiveValues(time.time(), np.nan)

  def _createPaintTimer(self, ) -> None:
    """Creates the update timer"""
    self.__paint_timer__ = QTimer()
    self.__paint_timer__.setInterval(33)
    self.__paint_timer__.setTimerType(Qt.TimerType.PreciseTimer)
    self.__paint_timer__.setSingleShot(True)
    self.__paint_timer__.timeout.connect(self.timedPaint)

  @paintTimer.GET
  def _getPaintTimer(self, **kwargs) -> QTimer:
    """Getter-function for paint timer"""
    if self.__paint_timer__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPaintTimer()
      return self._getPaintTimer(_recursion=True)
    if isinstance(self.__paint_timer__, QTimer):
      return self.__paint_timer__
    e = typeMsg('_paintTimer', self.__paint_timer__, QTimer)
    raise TypeError(e)

  def debugFunc01(self, ) -> None:
    """Start paint timer"""
    print('debug 01')
    print('starting dispatch test')
    # self._dispatchTest.start()
    self.timePlot.repaint()

  def timedPaint(self) -> None:
    """Handles timeout event"""
    if rospy.is_shutdown():
      print('Rospy was shut down')
      return
    self.timePlot.update()
    self.paintTimer.start()

  def initUI(self) -> None:
    """Reimplementation"""
    LayoutWindow.initUI(self)
    # self.paintTimer.start()
    # self._subscriber = subscriberFactory('/tool/pump_current',
    #                                      self.receiveData, )
