"""VerticalSpacer provides a vertically expanding spacer widget."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter, QBrush, QColor
from PySide6.QtWidgets import QSizePolicy
from vistutils.waitaminute import typeMsg

from ezros.gui.widgets import BaseWidget
from ezros.gui.factories import parseColor, emptyPen
from ezros.gui.shortnames import Fixed, Spread, SolidFill


class VerticalSpacer(FillWidget):
  """VerticalSpacer provides a vertically expanding spacer widget."""

  __fixed_width__ = None
  __fallback_width__ = 32
  __fill_brush__ = None
  __fill_color__ = None
  __fallback_color__ = QColor(255, 255, 255, 255)

  def getFillColor(self, ) -> QColor:
    """Returns the color"""
    if self.__fill_color__ is None:
      return self.__fallback_color__
    if isinstance(self.__fill_color__, QColor):
      return self.__fill_color__
    e = typeMsg('__fill_color__', self.__fill_color__, QColor)
    raise TypeError(e)

  def setFillColor(self, *args) -> None:
    """Sets the color"""
    color = parseColor(*args)
    if color is not None:
      if isinstance(color, QColor):
        self.__fill_color__ = color
      else:
        e = typeMsg('color', color, QColor)
        raise TypeError(e)
    else:
      e = """Unable to parse given arguments to color!"""
      raise ValueError(e)

  def getFillBrush(self) -> QBrush:
    """Returns the brush"""
    if self.__fill_brush__ is None:
      self.__fill_brush__ = QBrush()
      self.__fill_brush__.setStyle(SolidFill)
      self.__fill_brush__.setColor(self.getFillColor())
    return self.__fill_brush__

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)
    self.setFixedHeight(32)
    policy = QSizePolicy(Spread, Fixed)
    self.setSizePolicy(policy)
    color = parseColor(*args, **kwargs)
    if color is not None:
      self.setFillColor(color)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget"""
    painter = QPainter()
    painter.begin(self)
    painter.setBrush(self.getFillBrush())
    painter.setPen(emptyPen())
    painter.drawRect(event.rect())
    painter.end()
