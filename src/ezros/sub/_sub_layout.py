"""SubLayout organizes the windows on the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from ezros.gui.widgets import JKFlipFlop, DataWidget, LabelWidget, \
  VerticalLayout, HorizontalLayout, GridLayout, BaseWidget
from ezros.gui.windows import LayoutWindow
from morevistutils import Wait


class SubLayout(LayoutWindow):
  """SubLayout organizes the windows on the application"""

  baseWidget = Wait(BaseWidget, )

  baseLayout = Wait(VerticalLayout, )
  banner = Wait(LabelWidget, 'Welcome to EZRos!')

  controlWidget = Wait(BaseWidget, )
  controlPanel = Wait(HorizontalLayout, )

  robotId = Wait(LabelWidget, 'Robot ID: 69420')
  pumpToggle = Wait(JKFlipFlop, )
  statusIndicator = Wait(LabelWidget, 'Status: not yet implemented')
  pumpScenario = Wait(LabelWidget, 'Pump scenario: not yet implemented')

  dataLayout = Wait(GridLayout, )
  gridWidget = Wait(BaseWidget, )
  dataHeader = Wait(LabelWidget, )
  dataControl = Wait(LabelWidget, )
  data = Wait(DataWidget, )

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('Welcome to EZRos!')
    self.setMinimumSize(480, 320)

  def initUI(self, ) -> None:
    """Initializes the user interface"""
    self.baseLayout.addWidget(self.banner)
    self.controlPanel.addWidget(self.pumpToggle)
    self.controlPanel.addWidget(self.pumpScenario)
    self.controlPanel.addWidget(self.statusIndicator)
    self.controlPanel.addWidget(self.robotId)
    self.controlWidget.setLayout(self.controlPanel)
    self.baseLayout.addWidget(self.controlWidget)
    self.dataLayout.addWidget(self.dataHeader, 0, 1)
    self.dataLayout.addWidget(self.dataControl, 1, 0)
    self.dataLayout.addWidget(self.data, 1, 1)
    self.gridWidget.setLayout(self.dataLayout)
    self.baseLayout.addWidget(self.gridWidget)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
