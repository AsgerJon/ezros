"""Pinginator provides a widget representing the latency of a network
connection."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from subprocess import PIPE, run

from PySide6.QtCore import Qt, QRect, QPoint, QTimer, QMargins
from PySide6.QtGui import QPainter, \
  QBrush, \
  QColor, \
  QPaintEvent, \
  QPen, \
  QFontMetrics
from PySide6.QtWidgets import QSizePolicy
from ezside.core import SolidFill, SolidLine, emptyBrush, AlignCenter, Expand
from ezside.widgets import BaseWidget
from vistutils.waitaminute import typeMsg

from ezros.settings import Defaults


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
    out = float(res.stdout.split('time=')[1].split(' ms')[0])
    return int(round(out))

  def _update(self) -> None:
    """Update the latency."""
    self.__inner_latency__ = self._ping()
    fontMetrics = QFontMetrics(Defaults.getPingFont())
    margins = QMargins(4, 4, 4, 4)
    rect = fontMetrics.boundingRect('PING: 999 ms')
    self.setMinimumSize((rect + margins).size())

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
    if self.__inner_latency__ is None:
      self._update()
    brush = QBrush()
    brush.setStyle(SolidFill)
    if self.__inner_latency__ < 25:
      brush.setColor(QColor(0, 255, 0))
    elif self.__inner_latency__ < 50:
      brush.setColor(QColor(144, 255, 0))
    elif self.__inner_latency__ < 75:
      brush.setColor(QColor(192, 255, 0))
    elif self.__inner_latency__ < 100:
      brush.setColor(QColor(255, 255, 0))
    elif self.__inner_latency__ < 125:
      brush.setColor(QColor(255, 192, 0))
    elif self.__inner_latency__ < 150:
      brush.setColor(QColor(255, 144, 0))
    elif self.__inner_latency__ < 175:
      brush.setColor(QColor(255, 91, 0))
    else:
      brush.setColor(QColor(255, 0, 0))
    return brush

  def _getPen(self) -> QPen:
    """Getter-function for the pen."""
    pen = QPen()
    pen.setColor(QColor(0, 0, 0))
    pen.setWidth(1)
    pen.setStyle(SolidLine)
    return pen

  def _getText(self) -> str:
    """Getter-function for the text"""
    return 'PING: %d ms' % self.__inner_latency__

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the widget."""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    viewSize = viewRect.size()
    padRect = QRect(QPoint(0, 0), viewSize)
    padRect.moveCenter(viewRect.center())
    painter.setBrush(self._getBrush())
    painter.setPen(self._getPen())
    painter.drawRoundedRect(padRect, 10, 10)
    painter.setBrush(emptyBrush())
    painter.setPen(Defaults.getPingPen())
    font = Defaults.getPingFont()
    font.setPointSize(12)
    painter.setFont(font)
    text = self._getText()
    textRect = painter.boundingRect(padRect, AlignCenter, text)
    textRect.moveCenter(viewRect.center())
    painter.drawText(textRect, AlignCenter, text)
    painter.end()

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.setFixedSize(48, 32)
    self._getTimer().start()
    self.setSizePolicy(Expand, QSizePolicy.Policy.Preferred)
    self.update()
