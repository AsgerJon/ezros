"""The FillRect factory creates a function applying solid paint to the
underlying device"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QBrush, QColor, QPainter, QPaintEvent

from ezros.gui.factories import emptyPen
from ezros.gui.paint import AbstractPaint


class FillRect(AbstractPaint):
  """The FillRect factory creates a function applying solid paint to the
  underlying device"""

  def __init__(self, brush: QBrush) -> None:
    self.brush = brush

  def paintOp(self, event: QPaintEvent, painter: QPainter) -> None:
    """Fills the rect with the given brush"""
    painter.setBrush(self.brush)
    painter.setPen(emptyPen())
    painter.drawRect(event.rect())
