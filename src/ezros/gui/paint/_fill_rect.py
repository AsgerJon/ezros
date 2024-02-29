"""The FillRect factory creates a function applying solid paint to the
underlying device"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPainter, QPaintEvent

from ezros.gui.factories import emptyPen, parseBrush
from ezros.gui.paint import AbstractPaint
from ezros.gui.shortnames import Lime, Silver
from morevistutils import Wait


class FillRect(AbstractPaint):
  """The FillRect factory creates a function applying solid paint to the
  underlying device"""

  brush = Wait(parseBrush, Silver, )

  def __init__(self, *args, **kwargs) -> None:
    brush = parseBrush(*args, Silver, **kwargs)

  def paintOp(self, event: QPaintEvent, painter: QPainter) -> None:
    """Fills the rect with the given brush"""
    painter.setBrush(self.brush)
    painter.setPen(emptyPen())
    painter.drawRect(event.rect())
