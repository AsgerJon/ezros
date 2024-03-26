"""LineEdit wrapper.  """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLineEdit
from attribox import AttriBox
from ezside.widgets import BaseWidget
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.widgets import Horizontal, parseText


class _LineEdit(QLineEdit):
  """LineEdit wrapper.  """

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    QLineEdit.__init__(self, )
    text = maybe(parseText(*args, **kwargs), 'input text')
    if isinstance(text, str):
      self.setPlaceholderText(text)
    else:
      e = typeMsg('text', text, str)
      raise TypeError(e)


class LineEdit(BaseWidget):
  """LineEdit wrapper.  """

  cursorPositionChanged = Signal(int, int)
  editingFinished = Signal()
  inputRejected = Signal()
  returnPressed = Signal()
  selectionChanged = Signal()
  textChanged = Signal(str)
  textEdited = Signal(str)
  baseLayout = AttriBox[Horizontal]()
  lineEdit = AttriBox[QLineEdit]()

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
