"""TabPage provides the entries in the tab widget for the main window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QVBoxLayout
from attribox import AttriBox
from ezside.widgets import BaseWidget


class TabPage(BaseWidget):
  """TabPage provides the entries in the tab widget for the main window."""

  baseLayout = AttriBox[QVBoxLayout]()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setLayout(self.baseLayout)
