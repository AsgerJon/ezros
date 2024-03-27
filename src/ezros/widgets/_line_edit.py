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

from ezros.rosutils import EmptyField
from ezros.widgets import Horizontal


class _LineEdit(QLineEdit):
  """LineEdit wrapper.  """
  __fallback_placeholder__ = 'Enter text here'

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    QLineEdit.__init__(self, *args, **kwargs)

  def setPlaceholderText(self, placeholderText: str) -> None:
    """Set the placeholder text."""
    ic(self, placeholderText)
    QLineEdit.setPlaceholderText(self, placeholderText)


class LineEdit(BaseWidget):
  """LineEdit wrapper.  """
  __placeholder_text__ = None

  text = EmptyField()

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

  def __init__(self, placeHolder: str = None) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, )
    self.__placeholder_text__ = maybe(placeHolder, 'Enter text here')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.lineEdit.lineEdit.setPlaceholderText(self.__placeholder_text__)
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

  def setPlaceholderText(self, text: str) -> None:
    """Set the placeholder text."""
    self.lineEdit.setPlaceholderText(text)

  @text.GET
  def __str__(self) -> str:
    """String representation"""
    return QLineEdit.text(self.lineEdit)
