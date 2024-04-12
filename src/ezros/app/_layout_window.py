"""LayoutWindow organizes the layouts used in the application"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
import time

from attribox import AttriBox
from ezside.widgets import BaseWidget, Grid, Label, PushButton
from icecream import ic
from ezside.windows import BaseWindow

from ezros.widgets import Button

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Grid]()
  baseLabel = AttriBox[Label]('LMAO')
  pumpButton = AttriBox[Button]('Pump Control')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMinimumSize(1200, 600)
    self.baseLayout.addWidget(self.baseLabel, 0, 0)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
