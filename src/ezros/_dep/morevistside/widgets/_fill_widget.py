"""FillWidget provides a widget with a fill color"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PyQt5.QtGui import QBrush

from _dep.morevistside.core import parseBrush
from _dep.morevistside.widgets import BaseWidget
from _dep.morevistutils import FlexField


class FillWidget(BaseWidget):
  """FillWidget provides a widget with a fill color"""

  fillBrush = FlexField[QBrush](parseBrush, )

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)
    self.fillBrush = 144, 0, 255, 255
