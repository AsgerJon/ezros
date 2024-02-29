"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from ezros.gui.factories import parseBrush, textPen, parsePen, parseFont
from ezros.gui.paint import BorderRect, FillRect, TextRect
from ezros.gui.shortnames import Silver, Black
from ezros.gui.widgets import PaintWidget
from morevistutils import TextField, Wait


class LabelWidget(PaintWidget):
  """LabelWidget provides a text label."""

  __fallback_text__ = 'Label'

  innerText = TextField()
  backgroundFill = Wait(parseBrush, Silver)
  borderLine = Wait(parsePen, Black, 2)
  textPen = Wait(textPen, )
  textFont = Wait(parseFont, 'Arial', 10)

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    text = None
    for arg in args:
      if isinstance(arg, str) and text is None:
        text = arg
        break
    else:
      text = self.__fallback_text__
    self.innerText = text

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget"""
    painter = QPainter()
    painter.begin(self)
    text = self.innerText
    painter.end()
