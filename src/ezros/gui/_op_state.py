"""OpState provides a selection of operational states relevant for the
current support effort. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QComboBox
from vistside.widgets import BaseWidget, LabelWidget, BaseLayoutField, \
  LabelField
from vistutils.fields import Wait, FieldBox


class OpSelect(QComboBox):
  """Dropdown menu selecting operational state"""

  __content_lock__ = False
  newState = Signal(int)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the OpSelect."""
    QComboBox.__init__(self, *args, **kwargs)
    self.addItem('Air')
    self.addItem('Water')
    self.addItem('Paint')
    self.__content_lock__ = True
    self.connectActions()

  def addItem(self, *args, **kwargs) -> None:
    """Allows only air, water and paint to be added."""
    if self.__content_lock__:
      raise AttributeError('OpSelect instance locked!')
    QComboBox.addItem(self, *args, **kwargs)

  def connectActions(self) -> None:
    """Connects the widget's signals to its slots."""
    self.currentIndexChanged.connect(self.newState)


class OpState(BaseWidget):
  """OpState provides a selection of operational states relevant for the
  current support effort. """

  stateChange = Signal()
  currentIndexChanged = Signal(int)
  nowAir = Signal()
  nowWater = Signal()
  nowPaint = Signal()

  baseLayout = BaseLayoutField(layout='vertical')
  header = LabelField('Operational State')
  opSelect = FieldBox[OpSelect]()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new OpState."""
    BaseWidget.__init__(self, *args, **kwargs)
    self.initUI()
    self.connectActions()

  def initUI(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.header)
    self.baseLayout.addWidget(self.opSelect)
    self.setLayout(self.baseLayout)

  def connectActions(self) -> None:
    """Connects the widget's signals to its slots."""
    self.opSelect.newState.connect(self.currentIndexChanged)
    self.opSelect.newState.connect(self.stateChangeHandle)

  @Slot(int)
  def stateChangeHandle(self, index: int) -> None:
    """Handles the stateChange signal."""
    self.stateChange.emit()
    if index == 0:
      self.nowAir.emit()
    elif index == 1:
      self.nowWater.emit()
    elif index == 2:
      self.nowPaint.emit()
