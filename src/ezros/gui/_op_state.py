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
from vistutils.fields import Wait


class OpSelect(QComboBox):
  """Dropdown menu selecting operational state"""

  __content_lock__ = False
  newState = Signal(int)
  airContent = Signal()
  waterContent = Signal()
  paintContent = Signal()

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
    self.currentIndexChanged.connect(self.handleChange)

  @Slot(int)
  def handleChange(self, index: int) -> None:
    """Handles the change in index."""
    if 'air' in self.itemText(index).lower():
      self.airContent.emit()
    elif 'water' in self.itemText(index).lower():
      self.waterContent.emit()
    elif 'paint' in self.itemText(index).lower():
      self.paintContent.emit()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> OpSelect:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> OpSelect:
    """Applies the value to the field."""
    return self


class OpField(Wait):
  """The OpField class provides a descriptor for instances of OpSelect."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the OpField."""
    Wait.__init__(self, OpSelect, *args, **kwargs)


class OpState(BaseWidget):
  """OpState provides a selection of operational states relevant for the
  current support effort. """

  stateChange = Signal()
  currentIndexChanged = Signal(int)
  nowAir = Signal()
  nowWater = Signal()
  nowPaint = Signal()

  baseLayout = BaseLayoutField(layout='vertical')
  header = LabelField('Fluid Content')
  opSelect = OpField()

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

  @classmethod
  def getDefault(cls, *args, **kwargs) -> OpState:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> OpState:
    """Applies the arguments contained in value to the widget."""
    return self


class OpStateField(Wait):
  """The OpStateField class provides a descriptor for instances of
  OpState."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the OpStateField."""
    Wait.__init__(self, OpState, *args, **kwargs)
