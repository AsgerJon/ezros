"""TabWidget organizes the functionalities into tabs"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QTabWidget
from attribox import AttriBox
from icecream import ic

from ezros.widgets import LivePlot

ic.configureOutput(includeContext=True)


class TabWidget(QTabWidget):
  """TabWidget organizes the functionalities into tabs."""

  livePage = AttriBox[LivePlot]()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.livePage.initUi()
    self.addTab(self.livePage, 'Live')
    self.livePage.update()
    ic()
