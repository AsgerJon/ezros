"""TextPaint prints text on a widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QPaintEvent, QPainter, QFont, QPen

from ezros.gui.factories import parseFont
from ezros.gui.shortnames import Black
from ezros.gui.paint import AbstractPaint
from morevistutils.fields import WaitField


class TextPaint(AbstractPaint):
  """TextPaint prints text on a widget. """

  textFont = WaitField(QFont, parseFont, 'Courier', 24, )
  textPen = WaitField(QPen, )

  @textPen.CREATE
  def getTextPen(self, ) -> QPen:
    """Returns the pen for the text"""
    pen = QPen()
    pen.setColor(Black)
    pen.setWidth(1)
    pen.setStyle(Qt.PenStyle.SolidLine)
    return pen

  def applyPaint(self, event: QPaintEvent, painter: QPainter) -> None:
    """Paints the text"""
    painter.setFont(self.textFont)
    painter.setPen(self.textPen)
    painter.drawText(event.rect(), painter.device().innerText)
