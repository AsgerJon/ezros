"""LayoutWindow organizes the layouts used in the application"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from attribox import AttriBox
from ezside import BaseWindow
from ezside.widgets import BaseWidget, Grid, Label
from icecream import ic

from ezros.rosutils import LiveData
from ezros.widgets import RosToggle, Pinginator

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Grid]()
  header = AttriBox[Label]('LMAO')
  pinginator = AttriBox[Pinginator]()
  pumpControl = AttriBox[RosToggle]('Pump Control')
  sprayControl = AttriBox[RosToggle]('Spray Control')
  pumpData = AttriBox[LiveData]()
  pumpCurrentLabel = AttriBox[Label]('')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMinimumSize(640, 480)
    self.baseLayout.addWidget(self.header, 0, 0, 1, 3)
    self.baseLayout.addWidget(self.sprayControl, 1, 0, 1, 1)
    self.baseLayout.addWidget(self.pinginator, 1, 1, 1, 1)
    self.baseLayout.addWidget(self.pumpControl, 1, 2, 1, 1)
    self.baseLayout.addWidget(self.pumpCurrentLabel, 2, 0, 1, 3)
    self.baseLayout.addWidget(self.pumpData, 3, 0, 1, 3)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
