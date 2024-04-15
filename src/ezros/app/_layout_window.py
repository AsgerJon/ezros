"""LayoutWindow organizes the layouts used in the application"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
import time

from attribox import AttriBox
from ezside.widgets import BaseWidget, \
  Grid, \
  Label, \
  PushButton, \
  HorizontalSpacer
from icecream import ic
from ezside.windows import BaseWindow

from ezros.rosutils import RollingArray, LiveData
from ezros.widgets import Button, RosToggle, Pinginator

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Grid]()
  baseLabel = AttriBox[Label]('LMAO')
  activatePump = AttriBox[Button]('Pump ON')
  deactivatePump = AttriBox[Button]('Pump OFF')
  pumpStatus = AttriBox[Label]('Pump Status: OFF')
  pumpToggle = AttriBox[RosToggle]('/tool/pump_command')
  sprayToggle = AttriBox[RosToggle]('/tool/spray_command')
  hSpacer = AttriBox[HorizontalSpacer]()
  pinginator = AttriBox[Pinginator]()
  pumpData = AttriBox[LiveData]()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMinimumSize(640, 480)
    self.baseLayout.addWidget(self.sprayToggle, 0, 0, 1, 1)
    self.baseLayout.addWidget(self.pinginator, 0, 1, 1, 1)
    self.baseLayout.addWidget(self.pumpToggle, 0, 2, 1, 1)
    # self.baseLayout.addWidget(self.activatePump, 1, 0)
    # self.baseLayout.addWidget(self.deactivatePump, 1, 1)
    # self.baseLayout.addWidget(self.pumpStatus, 2, 0, 1, 2)
    self.baseLayout.addWidget(self.baseLabel, 1, 0, 1, 3)
    self.baseLayout.addWidget(self.pumpData, 2, 0, 1, 3)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
