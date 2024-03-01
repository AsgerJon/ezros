"""PushButton implementation"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.paint import TextRect, FillRect
from ezros.gui.widgets import PaintWidget
from morevistutils import TextField


class PushButton(PaintWidget):
  """PushButton implementation"""

  innerText = TextField()
  
  textFill = FillRect()
  borderRect = FillRect()
  textLabel = TextRect()
