"""ClockWidget provides a clock widget for the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from datetime import datetime

from PySide6.QtGui import QShowEvent

from morevistside.core import TimerField, Coarse, parseParent
from morevistside.widgets import BaseWidget


class ClockWidget(BaseWidget):
  """ClockWidget provides a clock widget for the main application window"""

  __update_timer__ = None
  __lcd_view__ = None
  __recent_update__ = None
  _latest = None
  timer = TimerField(200, Coarse, singleShot=False)

  def __init__(self, *args) -> None:
    parent = parseParent(*args)
    BaseWidget.__init__(self, parent)

  def _clockStep(self, *args, **kwargs) -> None:
    """Updates the clock"""
    rightNow = datetime.now()
    val = rightNow.second + rightNow.minute * 60 + rightNow.hour * 3600
    if val != self._latest:
      self._latest = val
      self.clock = rightNow.strftime("%H:%M:%S")
      self.update()

  def showEvent(self, event: QShowEvent, ):
    """Shows the widget"""
    self.timer.timeout.connect(self._clockStep)
    self.timer.start()
    BaseWidget.showEvent(self, event)

  def getDefault(self, *args, **kwargs, ) -> ClockWidget:
    """Creates a default instance"""
    return ClockWidget(*args, **kwargs)
