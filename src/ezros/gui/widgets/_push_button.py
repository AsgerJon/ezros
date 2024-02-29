"""PushButton implementation"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.paint import TextRect
from ezros.gui.widgets import PaintWidget


class PushButton(PaintWidget):
  """PushButton implementation"""

  label = TextRect()
