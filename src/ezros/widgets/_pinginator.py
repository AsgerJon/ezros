"""Pinginator provides a widget representing the latency of a network
connection."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from subprocess import PIPE, run

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QBrush
from ezside.core import SolidFill
from ezside.widgets import BaseWidget
from vistutils.waitaminute import typeMsg


class Pinginator(BaseWidget):
  """Pinginator provides a widget representing the latency of a network
  connection."""

  __inner_timer__ = None
  __inner_latency__ = None

  @staticmethod
  def _ping() -> int:
    """Ping the network."""
    res = run(['ping', '9.9.9.9', '-c', '1'],
              stdout=PIPE,
              stderr=PIPE,
              text=True)
    if res.returncode:
      e = str(res.stderr)
      raise RuntimeError(e)
    return int(res.stdout.split('time=')[1].split(' ms')[0])

  def _update(self) -> None:
    """Update the latency."""
    self.__inner_latency__ = self._ping()

  def _createTimer(self) -> None:
    """Create a timer."""
    if self.__inner_timer__ is not None:
      e = """Timer already exists."""
      raise AttributeError(e)
    self.__inner_timer__ = QTimer()
    self.__inner_timer__.setInterval(500)
    self.__inner_timer__.setTimerType(Qt.TimerType.VeryCoarseTimer)
    self.__inner_timer__.timeout.connect(self._update)

  def _getTimer(self, **kwargs) -> QTimer:
    """Get the timer."""
    if self.__inner_timer__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      kwargs['_recursion'] = True
      self._createTimer()
      return self._getTimer(**kwargs)
    if isinstance(self.__inner_timer__, QTimer):
      return self.__inner_timer__
    e = typeMsg('self.__inner_timer__', self.__inner_timer__, QTimer)
    raise TypeError(e)

  def _getBrush(self) -> QBrush:
    """Getter-function for state aware brush. """
    brush = QBrush()
    brush.setStyle(SolidFill)

  def paintEvent(self) -> None:
    """Paint the widget."""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()

    painter.end()
