"""PushButton wrapper. Allows for easy replacement."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.widgets import Vertical, parseText


class _PushButton(QPushButton):
  """PushButton wrapper. Allows for easy replacement."""

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    QPushButton.__init__(self, )
    text = maybe(parseText(*args, **kwargs), 'Click')
    if isinstance(text, str):
      self.setText(text)
    else:
      e = typeMsg('text', text, str)
      raise TypeError(e)


class PushButton(BaseWidget):
  """PushButton wrapper. Allows for easy replacement."""

  innerButton = AttriBox[_PushButton]()
  layout = AttriBox[Vertical]()
  clicked = Signal()
  leftClick = Signal()
  rightClick = Signal()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.layout.addWidget(self.innerButton)
    self.setLayout(self.layout)
    self.initActions()

  def initActions(self) -> None:
    """Initialize the actions."""
    self.innerButton.clicked.connect(self.clicked)
