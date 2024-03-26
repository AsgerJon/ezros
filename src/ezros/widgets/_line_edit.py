"""LineEdit wrapper.  """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLineEdit
from attribox import AttriBox, this
from ezside.widgets import BaseWidget
from icecream import ic
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.widgets import Horizontal, parseText


class _LineEdit(QLineEdit):
  """LineEdit wrapper.  """
  __fallback_placeholder__ = 'Enter text here'

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    QLineEdit.__init__(self, )
    for arg in args:
      if isinstance(arg, BaseWidget):
        placeholder = getattr(arg, '__placeholder_text__', None)
        if isinstance(placeholder, str):
          self.setPlaceholderText(placeholder)
          break
        if placeholder is None:
          self.setPlaceholderText(self.__fallback_placeholder__)
          break
        e = typeMsg('placeholder', placeholder, str)
        raise TypeError(e)
    else:
      for arg in args:
        if isinstance(arg, str):
          self.setPlaceholderText(arg)
          break
      else:
        self.setPlaceholderText(self.__fallback_placeholder__)

  def setPlaceholderText(self, placeholderText: str) -> None:
    """Set the placeholder text."""
    ic(self, placeholderText)
    QLineEdit.setPlaceholderText(self, placeholderText)


class LineEdit(BaseWidget):
  """LineEdit wrapper.  """
  __placeholder_text__ = None

  cursorPositionChanged = Signal(int, int)
  editingFinished = Signal()
  inputRejected = Signal()
  returnPressed = Signal()
  selectionChanged = Signal()
  textChanged = Signal(str)
  textEdited = Signal(str)
  baseLayout = AttriBox[Horizontal]()
  lineEdit = AttriBox[_LineEdit](this)
  placeholderText = AttriBox[str]()

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, **kwargs)
    placeholderText = parseText(*args, **kwargs)
    if isinstance(placeholderText, str):
      self.__placeholder_text__ = placeholderText
      
  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.lineEdit)
    self.setLayout(self.baseLayout)
    self.initActions()

  def initActions(self) -> None:
    """Initialize the actions."""
    self.lineEdit.cursorPositionChanged.connect(self.cursorPositionChanged)
    self.lineEdit.editingFinished.connect(self.editingFinished)
    self.lineEdit.inputRejected.connect(self.inputRejected)
    self.lineEdit.returnPressed.connect(self.returnPressed)
    self.lineEdit.selectionChanged.connect(self.selectionChanged)
    self.lineEdit.textChanged.connect(self.textChanged)
    self.lineEdit.textEdited.connect(self.textEdited)
