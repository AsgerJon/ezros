"""PushButton wrapper. Allows for easy replacement."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, QEvent
from PySide6.QtGui import QFont, QEnterEvent
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget
from vistutils.parse import maybe

from ezros.defaults import Settings
from ezros.widgets import Vertical


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

  holdOn = Signal()
  holdOff = Signal()

  __mouse_down__ = False

  def __init__(self, *args) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self)
    for arg in args:
      if isinstance(arg, str):
        self.setText(arg)
        break
    else:
      self.setText('CLICK')

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

  def mousePressEvent(self, event) -> None:
    """Mouse press event."""
    self.__mouse_down__ = True
    self.holdOn.emit()
    BaseWidget.mousePressEvent(self, event)

  def mouseReleaseEvent(self, event) -> None:
    """Mouse release event."""
    if self.__mouse_down__:
      self.holdOff.emit()
    self.__mouse_down__ = False
    BaseWidget.mouseReleaseEvent(self, event)

  def enterEvent(self, event: QEnterEvent):
    """Enter event."""
    self.__mouse_down__ = False
    BaseWidget.enterEvent(self, event)

  def leaveEvent(self, event: QEvent) -> None:
    """Leave event."""
    if self.__mouse_down__:
      self.holdOff.emit()
    self.__mouse_down__ = False
    BaseWidget.leaveEvent(self, event)
