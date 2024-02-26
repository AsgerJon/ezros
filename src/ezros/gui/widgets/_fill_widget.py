"""FillWidget fills the background of the widget with solid color"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.paint import FillRectPaint
from ezros.gui.widgets import PaintWidget


class FillWidget(PaintWidget):
  """FillWidget fills the background of the widget with solid color"""

  background = FillRectPaint()
