"""FillRectPaint fills the update rect with solid color."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QBrush, QColor

from ezros.gui.factories import parseBrush, emptyPen
from ezros.gui.paint import AbstractPaint
from ezros.gui.shortnames import Lime, SolidFill
from morevistutils.fields import WaitField


class FillRectPaint(AbstractPaint):
  """FillRectPaint fills the update rect with solid color."""

  fillBrush = WaitField(QBrush, parseBrush, Lime, SolidFill, )

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaint.__init__(self, *args, **kwargs)

  def applyPaint(self, event, painter) -> None:
    """Paints the background"""
    painter.setBrush(self.fillBrush)
    painter.setPen(emptyPen())
    painter.drawRect(event.rect())
