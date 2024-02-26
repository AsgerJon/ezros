"""BorderRectPaint draws a border around the update rectangle, without
fill."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPen

from ezros.gui.factories import parsePen, emptyBrush
from ezros.gui.paint import AbstractPaint
from ezros.gui.shortnames import Black, SolidLine, SteelBlue
from morevistutils.fields import WaitField


class BorderRectPaint(AbstractPaint):
  """BorderRectPaint draws a border around the update rectangle, without
  fill."""

  borderPen = WaitField(QPen, parsePen, SteelBlue, SolidLine, )

  def applyPaint(self, event, painter) -> None:
    """Paints the background"""
    painter.setPen(self.borderPen)
    painter.setBrush(emptyBrush())
    painter.drawRect(event.rect())
