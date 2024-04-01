"""This module provides expanding spacers preventing widgets from being
spread to fill up available space. Three versions are provided,
HorizontalSpacer, VerticalSpacer, and GridSpacer, expanding in the
horizontal, vertical, and both directions respectively. The spacers are
intended to be invisible except for during development. Use the visibility
flag in the settings."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QSize
from PySide6.QtGui import QPainter, QPaintEvent, QPen, QColor
from ezside.core import solidBrush, \
  Yellow, \
  Expand, \
  Tight, \
  emptyBrush, \
  DashLine
from ezside.core import dashPen
from ezside.widgets import BaseWidget

from ezros.defaults import Settings


class AbstractSpacer(BaseWidget):
  """AbstractSpacer class provides a base class for the spacers."""

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)
    self.setContentsMargins(0, 0, 0, 0)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the spacer."""
    if not Settings.spacerVisibility:
      return
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    painter.setPen(dashPen())
    painter.setBrush(solidBrush(Yellow))
    painter.drawRect(viewRect)
    painter.end()


class HorizontalSpacer(AbstractSpacer):
  """HorizontalSpacer class provides a horizontal spacer."""

  def __init__(self, *args, **kwargs) -> None:
    AbstractSpacer.__init__(self, *args, **kwargs)
    self.setSizePolicy(Expand, Tight)


class VerticalSpacer(AbstractSpacer):
  """VerticalSpacer class provides a vertical spacer."""

  def __init__(self, *args, **kwargs) -> None:
    AbstractSpacer.__init__(self, *args, **kwargs)
    self.setSizePolicy(Tight, Expand)


class GridSpacer(AbstractSpacer):
  """GridSpacer class provides a grid spacer."""

  def __init__(self, *args, **kwargs) -> None:
    AbstractSpacer.__init__(self, *args, **kwargs)
    self.setSizePolicy(Expand, Expand)


class HorizontalSeparator(HorizontalSpacer):
  """HorizontalSeparator class provides a horizontal separator."""

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the separator."""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    painter.setPen(dashPen())
    painter.setBrush(emptyBrush())
    y = viewRect.center().y()
    left, right = viewRect.left(), viewRect.right()
    painter.drawLine(left, y, right, y)
    painter.end()


class VerticalSeparator(VerticalSpacer):
  """VerticalSeparator class provides a vertical separator."""

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the separator."""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    pen = QPen()
    pen.setColor(QColor(192, 225, 255, 255))
    pen.setStyle(DashLine)
    pen.setWidth(2)
    painter.setPen(pen)
    painter.setBrush(emptyBrush())
    x = viewRect.center().x()
    top, bottom = viewRect.top(), viewRect.bottom()
    painter.drawLine(x, top, x, bottom)
    painter.end()