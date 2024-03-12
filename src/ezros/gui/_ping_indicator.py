"""PingIndicator visualizes latency by combining the ping with a color
ranging from green (low ping) to red (high ping). The widget simulates
ping value changes and visualizes these changes in real-time.
"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Slot
from PySide6.QtGui import QPaintEvent, QPainter, QBrush, QColor, QFont
from vistside.core import SolidFill, parseFont, Center
from vistside.widgets import BaseWidget
from vistutils.fields import Wait


class PingIndicator(BaseWidget):
  """
  A custom widget that displays ping values with a color gradient from green
  (low ping) to red (high ping). The widget simulates ping value changes and
  visualizes these changes in real-time. Lower ping values are displayed in
  green, indicating good connectivity, whereas higher values gradually shift
  to red, indicating poorer connectivity.
  """

  __ping_latency__ = None

  def __init__(self, *args, **kwargs) -> None:
    """
    Initializes the PingDisplayWidget with a default ping value and starts
    the ping simulation.

    Parameters:
        parent (Optional[QWidget]): The parent widget. Defaults to None.
    """
    BaseWidget.__init__(self, *args, **kwargs)
    self.__ping_latency__ = 1000  # Initial ping value
    self.setMinimumSize(200, 50)

  @Slot(int)
  def setPing(self, newPing: int) -> None:
    """Setter-function for the ping attribute."""
    self.__ping_latency__ = newPing

  def getPing(self, ) -> int:
    """Getter-function for the ping attribute"""
    if self.__ping_latency__ is None:
      return 1000
    if isinstance(self.__ping_latency__, int):
      return min([1000, self.__ping_latency__])

  def getBrush(self) -> QBrush:
    """Getter-function for the brush attribute"""
    r0, g0, b0 = 0, 255, 0
    r1, g1, b1 = 255, 0, 0
    f = self.getPing() / 1000
    r = int(r0 * (1 - f) + r1 * f)
    g = int(g0 * (1 - f) + g1 * f)
    b = int(b0 * (1 - f) + b1 * f)
    color = QColor(r, g, b)
    brush = QBrush()
    brush.setColor(color)
    brush.setStyle(SolidFill)
    return brush

  def getFont(self) -> QFont:
    """Getter-function for the font attribute"""
    font = parseFont('Arial', 24)
    if self.getPing() > 200:
      font.setBold(True)
    return font

  def paintEvent(self, event: QPaintEvent) -> None:
    """
    Handles the widget's painting event by creating a QPainter and calling
    the drawPing method to render the ping value.

    Parameters:
        event (QPaintEvent): The painting event.
    """
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    viewRect = painter.viewport()
    painter.setPen(self.emptyPen)
    painter.setBrush(self.getBrush())
    painter.drawRoundedRect(viewRect, 10, 10)
    painter.setFont(self.getFont())
    painter.setPen(self.textPen)
    text = 'Ping: %s ms' % self.getPing()
    painter.drawText(viewRect, Center, text)
    painter.end()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> PingIndicator:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> PingIndicator:
    """Applies the value to the field."""
    return self


class PingIndicatorField(Wait):
  """The PingIndicatorField class provides a descriptor for instances of
  PingIndicator."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the PingIndicatorField."""
    Wait.__init__(self, PingIndicator, *args, **kwargs)