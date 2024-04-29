"""ControlPeriodic provides a widget with two sliders controlling the on
and off periods of a periodic signal."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from attribox import AttriBox
from ezside.widgets import Vertical, HorizontalSlider, BaseWidget

from ezros.widgets import RosToggle


class ControlPeriodic(BaseWidget):
  """ControlPeriodic provides a widget with two sliders controlling the on
  and off periods of a periodic signal."""

  baseLayout = AttriBox[Vertical]()
  onPeriod = AttriBox[HorizontalSlider]('ON')
  offPeriod = AttriBox[HorizontalSlider]('OFF')
  toggle = AttriBox[RosToggle]()

  onValueChanged = Signal(float)
  offValueChanged = Signal(float)
  activated = Signal()
  deactivated = Signal()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.onPeriod)
    self.baseLayout.addWidget(self.offPeriod)
    self.baseLayout.addWidget(self.toggle)
    self.setLayout(self.baseLayout)

  def connectActions(self) -> None:
    """Connect the actions to the signals."""
    self.onPeriod.valueChanged.connect(self.onValueChanged)
    self.offPeriod.valueChanged.connect(self.offValueChanged)
    self.toggle.activated.connect(self.activated)
    self.toggle.deactivated.connect(self.deactivated)
