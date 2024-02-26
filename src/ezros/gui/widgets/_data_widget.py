"""DataWidget provides data plotting on a widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.paint import BorderRectPaint, FillRectPaint


class DataWidget:
  """DataWidget provides data plotting on a widget"""

  fillRect = FillRectPaint()
  borderRect = BorderRectPaint()
