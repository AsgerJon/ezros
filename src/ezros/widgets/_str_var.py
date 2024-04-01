"""StrVar provides a widget for setting a string variable."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget
from vistutils.waitaminute import typeMsg

from ezros.widgets import Grid, TightLabel, LineEdit


class StrVar(BaseWidget):
  """StrVar provides a widget for setting a string variable."""

  __fallback_value__ = None
  __inner_value__ = None

  baseLayout = AttriBox[Grid]()
  label = AttriBox[TightLabel]()
  lineEdit = AttriBox[LineEdit]()
  button = AttriBox[QPushButton]()

  clicked = Signal()

  valueChangedFull = Signal(str, str)
  valueChangedNew = Signal(str)
  valueChangedOld = Signal(str)
  valueChanged = Signal()

  valueRequestedFull = Signal(str)
  valueRequested = Signal()

  valueClearedFull = Signal(str)
  valueCleared = Signal()

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, **kwargs)
    initValue = kwargs.get('initValue', None)
    if initValue is None:
      for arg in args:
        if isinstance(arg, str):
          self.__fallback_value__ = arg
          break
    elif isinstance(initValue, str):
      self.__fallback_value__ = initValue
    else:
      e = typeMsg('initValue', initValue, str)
      raise TypeError(e)

  def initUi(self) -> None:
    """Initialize the widget."""
    self.baseLayout.addWidget(self.label)
    self.baseLayout.addWidget(self.lineEdit)
    self.baseLayout.addWidget(self.button)
    self.setLayout(self.baseLayout)

  def connectActions(self) -> None:
    """Initialize the actions."""

  def setValue(self, newValue: str) -> None:
    """Set the value."""
    if isinstance(newValue, str):
      self.__inner_value__ = newValue
      self.lineEdit.setText(newValue)
      self.valueChangedFull.emit(newValue, newValue)
      self.valueChangedNew.emit(newValue)
      self.valueChangedOld.emit(newValue)
      self.valueChanged.emit()
    else:
      e = typeMsg('newValue', newValue, str)
      raise TypeError(e)

  def getValue(self, **kwargs) -> str:
    """Get the value."""
    if self.lineEdit.text() != self.__inner_value__:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.update()
      return self.getValue(_recursion=True)
    return self.__inner_value__

  def clearValue(self) -> None:
    """Clear the value."""
    oldValue = self.__inner_value__
    self.__inner_value__ = None
    self.lineEdit.setText('')
    self.valueClearedFull.emit(oldValue)
    self.valueCleared.emit()
    self.update()

  def __get__(self, *_args, **_kwargs) -> str:
    """Get the value."""
    return self.getValue()

  def __set__(self, _, value: str) -> None:
    """Set the value."""
    self.setValue(value)
