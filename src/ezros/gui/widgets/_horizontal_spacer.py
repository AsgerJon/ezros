"""The HorizontalSpacer class provides widget that stretches horizontally,
but having fixed vertical height."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from PySide6.QtWidgets import QSizePolicy

from ezros.gui.factories import parseBrush, parseColor, emptyPen
from ezros.gui.shortnames import Fixed, Spread
from ezros.gui.widgets import PaintWidget
from morevistutils import Wait


class HorizontalSpacer(PaintWidget):
  """The HorizontalSpacer class provides widget that stretches horizontally,
  but having fixed vertical height."""

  __fixed_height__ = None
  __fallback_height__ = 32

  brush = Wait(parseBrush)

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    self.setFixedHeight(32)
    policy = QSizePolicy(Spread, Fixed)
    self.setSizePolicy(policy)
    color = parseColor(*args, **kwargs)
    if color is not None:
      self.brush.setColor(color)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget"""
    painter = QPainter()
    painter.begin(self)
    painter.setBrush(self.brush)
    painter.setPen(emptyPen())
    painter.drawRect(event.rect())
    painter.end()
