"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from ezros.gui.paint import BorderRect, FillRect, TextRect
from ezros.gui.widgets import PaintWidget
from morevistutils import TextField


class LabelWidget(PaintWidget):
  """LabelWidget provides a text label."""

  __fallback_text__ = 'Label'

  innerText = TextField()
  outerFill = FillRect()
  outerLine = BorderRect()
  innerFill = FillRect()
  innerLine = BorderRect()
  textPaint = TextRect()

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    text = None
    for arg in args:
      if isinstance(arg, str) and text is None:
        text = arg
        break
    else:
      text = self.__fallback_text__

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""
    self.outerFill.paintOp(event, painter)
    self.outerLine.paintOp(event, painter)
    self.innerFill.paintOp(event, painter)
    self.innerLine.paintOp(event, painter)
    self.textPaint.paintOp(event, painter, self.innerText)
