"""StrVar provides a widget for setting a string variable."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, QSize
from PySide6.QtGui import QResizeEvent
from PySide6.QtWidgets import QPushButton, QSizePolicy, QWidget
from attribox import AttriBox
from ezside.core import Tight
from ezside.widgets import BaseWidget
from icecream import ic
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.widgets import LineEdit, Label, Horizontal, HorizontalSpacer

ic.configureOutput(includeContext=True, )


class Header(Label):
  """Wrapper class that includes a formatting on the label."""

  __fallback_format_spec__ = 'Header: %s'
  __format_spec__ = None
  __inner_raw_text__ = None

  def setFormatSpec(self, formatSpec: str) -> None:
    """Set the format spec."""
    if isinstance(formatSpec, str):
      try:
        res = formatSpec % 'test'
      except Exception as e:
        ic(e)
        raise e
      self.__format_spec__ = formatSpec
    else:
      e = typeMsg('formatSpec', formatSpec, str)
      raise TypeError(e)

  def getFormatSpec(self, ) -> str:
    """Get the format spec."""
    fallback = getattr(self, '__fallback_format_spec__', )
    formatSpec = getattr(self, '__format_spec__', fallback)
    if formatSpec is None:
      e = """The format spec has not been set!"""
      raise AttributeError(e)
    if isinstance(formatSpec, str):
      return formatSpec
    e = typeMsg('formatSpec', formatSpec, str)
    raise TypeError(e)

  def getText(self, ) -> str:
    """Get the value."""
    return self.getFormatSpec() % Label.getText(self)


class StrVar(BaseWidget):
  """StrVar provides a widget for setting a string variable."""

  __fallback_value__ = None
  __inner_value__ = None

  __variable_name__ = None
  __button_text__ = 'Submit'

  baseLayout = AttriBox[Horizontal]()
  label = AttriBox[Label]()
  lineEdit = AttriBox[LineEdit]()
  button = AttriBox[QPushButton]()
  h1 = AttriBox[HorizontalSpacer]()

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
    nameKwarg = kwargs.get('name', None)
    placeholderKwarg = kwargs.get('placeholder', None)
    initValueKwarg = kwargs.get('initValue', None)
    buttonKwarg = kwargs.get('button', None)
    strArgs = [arg for arg in args if isinstance(arg, str)]
    strArgs = [*strArgs, None, None, None, None][:4]
    nameArg, placeholderArg, initValueArg, buttonArg = strArgs
    name = maybe(nameKwarg, nameArg)
    if name is not None:
      if isinstance(name, str):
        self.__variable_name__ = name
      else:
        e = typeMsg('name', name, str)
        raise TypeError(e)
    else:
      e = """Unable to parse required argument 'name'!"""
      raise ValueError(e)
    placeholder = maybe(placeholderKwarg, placeholderArg)
    if placeholder is not None:
      if isinstance(placeholder, str):
        self.lineEdit.setPlaceholderText(placeholder)
      else:
        e = typeMsg('placeholder', placeholder, str)
        raise TypeError(e)
    initValue = maybe(initValueKwarg, initValueArg)
    if initValue is not None:
      if isinstance(initValue, str):
        self.__inner_value__ = initValue
        self.lineEdit.setText(initValue)
      else:
        e = typeMsg('initValue', initValue, str)
        raise TypeError(e)
    button = maybe(buttonKwarg, buttonArg, self.__button_text__)
    if button is not None:
      if isinstance(button, str):
        self.button.setText(button)
      else:
        e = typeMsg('button', button, str)
        raise TypeError(e)
    self.label.text = '%s: unnamed' % self.__variable_name__

  def initUi(self) -> None:
    """Initialize the widget."""
    self.setSizePolicy(QSizePolicy.Policy.Preferred, Tight)
    self.setMinimumWidth(600)
    self.baseLayout.addWidget(self.label)
    self.baseLayout.addWidget(self.lineEdit)
    self.button.setSizePolicy(Tight, Tight)
    self.baseLayout.addWidget(self.button)
    self.baseLayout.addWidget(self.h1)
    self.setLayout(self.baseLayout)

  def connectActions(self) -> None:
    """Initialize the actions."""
    self.button.clicked.connect(self._lineEditToValue)
    self.button.clicked.connect(self.update)
    self.lineEdit.returnPressed.connect(self._lineEditToValue)
    self.lineEdit.returnPressed.connect(self.update)
    self.lineEdit.textEdited.connect(self.update)

  def setValue(self, newValue: str) -> None:
    """Set the value."""
    if isinstance(newValue, str):
      if newValue == self.__inner_value__:
        return
      self.__inner_value__ = newValue
      labelText = '%s: %s' % (self.__variable_name__, newValue)
      self.label.text = labelText
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

  def _lineEditToValue(self, ) -> None:
    """Set the value."""
    newValue = self.lineEdit.text
    self.setValue(newValue)

  def update(self, ) -> None:
    """Before invoking parent method, disables the button if the current
    value on the line edit matches the inner value. """
    if self.lineEdit.text == self.__inner_value__:
      if self.button.isEnabled():
        self.button.setEnabled(False)
        self.button.update()
    else:
      if not self.button.isEnabled():
        self.button.setEnabled(True)
        self.button.update()
    labelWidth = self.label.geometry().width()
    lineEditWidth = self.lineEdit.geometry().width()
    buttonWidth = self.button.geometry().width()
    labelHeight = self.label.geometry().height()
    lineEditHeight = self.lineEdit.geometry().height()
    buttonHeight = self.button.geometry().height()
    minHeight = max(labelHeight, lineEditHeight, buttonHeight)
    minWidth = labelWidth + lineEditWidth + buttonWidth
    self.setMinimumSize(QSize(minWidth, minHeight))
    self.adjustSize()
    self.parent().adjustSize()
    BaseWidget.update(self)
  #
  # def resizeEvent(self, event: QResizeEvent) -> None:
  #   """Resize the widget."""
  #   self.update()
  #   parentWidget = self.parent()
  #   while not isinstance(parentWidget, QWidget):
  #     parentWidget = parentWidget.parent()
  #     parentWidget.adjustSize()
  #   BaseWidget.resizeEvent(self, event)
