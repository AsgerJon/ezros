"""StrInput widget for inputting text. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from attribox import AttriBox, scope
from ezside.widgets import BaseWidget
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.rosutils import EmptyField
from ezros.widgets import Horizontal, TightLabel, LineEdit, PushButton

InputVals = tuple[str, Optional[str], Optional[str], Optional[str]]


class StrInput(BaseWidget):
  """StrInput widget for inputting text. """

  __fallback_label__ = 'Input'
  __fallback_button__ = 'Submit'
  __fallback_current__ = 'Current'
  __fallback_placeholder__ = 'Enter text here'

  text = EmptyField()

  baseLayout = AttriBox[Horizontal]()
  titleLabel = AttriBox[TightLabel]()
  lineEdit = AttriBox[LineEdit]()
  currentLabel = AttriBox[TightLabel]()
  updateButton = AttriBox[PushButton]()

  newValue = Signal(str, str)
  valueChanged = Signal()

  @staticmethod
  def typeGuard(self,
                title: Optional[str],
                button: Optional[str] = None,
                current: Optional[str] = None,
                placeholder: Optional[str] = None) -> InputVals:
    """Type guard."""
    if isinstance(title, str):
      if button is not None:
        if not isinstance(button, str):
          e = typeMsg('button', button, str)
          raise TypeError(e)
      if current is not None:
        if not isinstance(current, str):
          e = typeMsg('current', current, str)
          raise TypeError(e)
      if placeholder is not None:
        if not isinstance(placeholder, str):
          e = typeMsg('placeholder', placeholder, str)
          raise TypeError(e)
      return title, button, current, placeholder
    if title is None:
      raise ValueError('No title provided')
    e = typeMsg('title', title, str)
    raise TypeError(e)

  @classmethod
  def parseThis(cls, *args, **kwargs) -> InputVals:
    """Parses instances of BaseWidget"""
    for arg in args:
      if isinstance(arg, BaseWidget):
        widget = arg
        break
    else:
      raise ValueError('No BaseWidget found')
    title = getattr(widget, '__label_title__', None)
    button = getattr(widget, '__button_text__', None)
    current = getattr(widget, '__current_text__', None)
    placeholder = getattr(widget, '__placeholder_text__', None)
    return cls.typeGuard(title, button, current, placeholder)

  @classmethod
  def parsePosArgs(cls, *args) -> InputVals:
    """Parses positional arguments."""
    strArgs = [arg for arg in args if isinstance(arg, str)]
    title, button, current, placeholder = [*strArgs, *([None] * 4,)][:4]
    return cls.typeGuard(title, button, current, placeholder)

  @classmethod
  def parseKeyArgs(cls, **kwargs) -> InputVals:
    """Parses keyword arguments."""
    title = kwargs.get('label', None)
    button = kwargs.get('button', None)
    current = kwargs.get('current', None)
    placeholder = kwargs.get('placeholder', None)
    return cls.typeGuard(title, button, current, placeholder)

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, **kwargs)
    title, button, current, placeholder = None, None, None, None
    try:
      title, button, current, placeholder = self.parseThis(*args, **kwargs)
    except ValueError as valueError:
      title, button, current, placeholder = self.parsePosArgs(*args)
    self.__label_title__ = title
    self.__button_text__ = maybe(button, self.__fallback_button__)
    self.__current_text__ = maybe(current, self.__fallback_current__)
    self.__placeholder_text__ = maybe(placeholder,
                                      self.__fallback_placeholder__)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.titleLabel.setText(self.__label_title__)
    self.updateButton.setText(self.__button_text__)
    self.lineEdit.setPlaceholderText(self.__placeholder_text__)
    self.currentLabel.setText(self.__current_text__)
    self.baseLayout.addWidget(self.titleLabel)
    self.baseLayout.addWidget(self.lineEdit)
    self.baseLayout.addWidget(self.updateButton)
    self.baseLayout.addWidget(self.currentLabel)
    self.setLayout(self.baseLayout)

  def initActions(self) -> None:
    """Initialize the actions."""
    self.updateButton.clicked.connect(self.handleButton)
    self.lineEdit.returnPressed.connect(self.handleButton)

  def handleButton(self, ) -> None:
    """Handle the button."""
    self.valueChanged.emit()
    oldValue = self.currentLabel.text()
    newValue = self.lineEdit.text()
    self.newValue(newValue, oldValue)
    self.valueChanged.emit()
    self.currentLabel.setText(newValue)
    self.currentLabel.update()
    self.lineEdit.clear()

  @text.GET
  def __str__(self) -> str:
    """Return the string representation."""
    return self.currentLabel.text()
