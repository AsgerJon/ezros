"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPainter, QPaintEvent
from vistutils.fields import TypedField

from ezros.gui.factories import parseFont, textPen
from morevistside.widgets import FillWidget


class LabelWidget(FillWidget):
  """LabelWidget provides a text label."""

  innerText = TypedField(str, 'hello world', supportInit=True)

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the label."""
    FillWidget.paintEvent(self, event)
    painter = QPainter()
    painter.begin(self)
    baseFont = parseFont('Courier', 24, )
    fontPen = textPen()
    painter.setFont(baseFont)
    painter.setPen(fontPen)
    painter.drawText(self.rect(), self.innerText)
    painter.end()
