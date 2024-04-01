"""LayoutWindow organizes the layouts used in the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from attribox import AttriBox
from ezside.widgets import BaseWidget
from icecream import ic

from ezside.windows import BaseWindow
from ezros.widgets import CommandControl, \
  Horizontal, \
  DynWidget, \
  VerticalSeparator

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Horizontal]()
  dynamicWidget = AttriBox[DynWidget]()
  v1 = AttriBox[VerticalSeparator]()
  pumpWidget = AttriBox[CommandControl]('/tool/pump_command')
  v2 = AttriBox[VerticalSeparator]()
  sprayWidget = AttriBox[CommandControl]('/tool/spray_command')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.dynamicWidget)
    self.baseLayout.addWidget(self.v1)
    self.baseLayout.addWidget(self.pumpWidget)
    self.baseLayout.addWidget(self.v2)
    self.baseLayout.addWidget(self.sprayWidget)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
