"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.paint import TextPaint, FillRectPaint, BorderRectPaint
from ezros.gui.widgets import PaintWidget
from morevistutils.fields import TextField


class LabelWidget(PaintWidget):
  """LabelWidget provides a text label."""

  innerText = TextField('LOL')
  outerFill = FillRectPaint()
  outerLine = BorderRectPaint()
  innerFill = FillRectPaint()
  innerLine = BorderRectPaint()
  textPaint = TextPaint()
