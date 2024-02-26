"""BorderRect draws an outline on the update rectangle, without fill."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPen

from ezros.gui.factories import emptyBrush
from ezros.gui.paint import AbstractPaint


class BorderRect(AbstractPaint):
  """BorderRect draws an outline on the update rectangle, without fill."""

  def __init__(self, pen: QPen) -> None:
    self.pen = pen

  def paintOp(self, event, painter) -> None:
    """Applies the paint operation"""
    painter.setPen(self.pen)
    painter.setBrush(emptyBrush())
    painter.drawRect(event.rect())
   