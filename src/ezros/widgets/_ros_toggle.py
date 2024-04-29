"""RosToggle provides toggle control and indication of a particular dual
state topic."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, Slot
from attribox import AttriBox
from ezside.widgets import BaseWidget, Button, Label, Grid
from vistutils.waitaminute import typeMsg


class RosToggle(BaseWidget):
  """RosToggle provides toggle control and indication of a particular dual
  state topic. """

  __inner_state__ = False

  baseLayout = AttriBox[Grid]()
  header = AttriBox[Label]()
  onButton = AttriBox[Button]('ON')
  indicator = AttriBox[Label]('')
  offButton = AttriBox[Button]('OFF')

  activated = Signal()
  deactivated = Signal()
  stateChanged = Signal(bool)

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the RosToggle."""
    for arg in args:
      if isinstance(arg, str):
        self.header.setText(arg)
    BaseWidget.__init__(self, *args, **kwargs)

  def initUi(self, ) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.header, 0, 0, 1, 3)
    self.baseLayout.addWidget(self.onButton, 1, 0)
    self.baseLayout.addWidget(self.indicator, 1, 1)
    self.baseLayout.addWidget(self.offButton, 1, 2)
    self.setLayout(self.baseLayout)

  @Slot()
  def activate(self) -> None:
    """Activate the toggle."""
    if self.__inner_state__:
      return
    self.onButton.setEnabled(False)
    self.offButton.setEnabled(True)
    self.indicator.setText('ON')
    self.__inner_state__ = True
    self.stateChanged.emit(True)
    self.activated.emit()
    self.update()

  @Slot()
  def deactivate(self) -> None:
    """Deactivate the toggle."""
    if not self.__inner_state__:
      return
    self.onButton.setEnabled(True)
    self.offButton.setEnabled(False)
    self.indicator.setText('OFF')
    self.__inner_state__ = False
    self.stateChanged.emit(False)
    self.deactivated.emit()
    self.update()

  def connectActions(self) -> None:
    """Connect the actions to the signals."""
    self.onButton.clicked.connect(self.activate)
    self.offButton.clicked.connect(self.deactivate)
