"""LayoutWindow organizes the layouts used in the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
import time

from attribox import AttriBox
from ezside.widgets import BaseWidget
from icecream import ic
from ezside.windows import BaseWindow

from ezros.rosutils._deez_charts import ComplexDataChartWidget
from ezros.widgets import CommandControl, Label
from ezros.widgets import DynWidget, VerticalSeparator, Grid, TimerWidget

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """
  __right_now__ = None

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Grid]()
  dynamicWidget = AttriBox[ComplexDataChartWidget]()
  # timerWidget = AttriBox[TimerWidget]()
  v1 = AttriBox[VerticalSeparator]()
  pumpWidget = AttriBox[CommandControl]('/tool/pump_command')
  v2 = AttriBox[VerticalSeparator]()
  sprayWidget = AttriBox[CommandControl]('/tool/spray_command')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.__right_now__ = time.time()
    self.baseLayout.addWidget(self.dynamicWidget, 0, 0, 2, 1)
    self.baseLayout.addWidget(self.v1, 0, 1, 2, 1)
    self.baseLayout.addWidget(self.pumpWidget, 0, 2, 2, 1)
    self.baseLayout.addWidget(self.v2, 0, 3, 2, 1)
    self.baseLayout.addWidget(self.sprayWidget, 0, 4, 2, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
