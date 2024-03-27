"""PushButton wrapper. Allows for easy replacement."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.defaults import Settings
from ezros.widgets import Vertical, parseText


class _PushButton(QPushButton):
  """PushButton wrapper. Allows for easy replacement."""

  __fallback_font__ = Settings.getButtonFont()

  def __init__(self, text: str = None, font: QFont = None) -> None:
    """Initialize the widget."""
    QPushButton.__init__(self, )
    self.setText(maybe(text, 'Click'))
    self.setFont(maybe(font, QFont('Courier', 12)))


class PushButton(BaseWidget):
  """PushButton wrapper. Allows for easy replacement."""

  innerButton = AttriBox[_PushButton]()
  layout = AttriBox[Vertical]()
  clicked = Signal()
  leftClick = Signal()
  rightClick = Signal()

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.layout.addWidget(self.innerButton)
    self.setLayout(self.layout)
    # self.initActions()

  def connectActions(self) -> None:
    """Initialize the actions."""
    self.innerButton.clicked.connect(self.clicked)

  def setText(self, text: str) -> None:
    """Set the text."""
    self.innerButton.setText(text)
