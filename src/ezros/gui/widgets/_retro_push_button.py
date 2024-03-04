"""PushButton provides a mouse aware label widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.widgets import BaseWidget

from PySide6.QtGui import QPainter, QMouseEvent, QEnterEvent, \
  QPaintEvent
from PySide6.QtCore import Signal, Qt

from _dep.morevistutils import TextField


class RetroPushButton(BaseWidget):
  """
  A custom QWidget subclass that implements a retro-like push
  button.
  """
  clicked = Signal()
  innerText = TextField('CLICK')

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)
    self.setFixedSize(100, 50)
    self.isPressed = False
    self.isHovered = False

  def paintEvent(self, event: QPaintEvent) -> None:
    painter = QPainter(self)
    if self.isPressed:
      painter.setBrush(Qt.darkGray)
    elif self.isHovered:
      painter.setBrush(Qt.lightGray)
    else:
      painter.setBrush(Qt.gray)
    painter.drawRect(self.rect())

  def mousePressEvent(self, event: QMouseEvent) -> None:
    if event.button() == Qt.LeftButton:
      self.isPressed = True
      self.update()

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    if event.button() == Qt.LeftButton and self.isPressed:
      self.isPressed = False
      self.update()
      self.clicked.emit()

  def enterEvent(self, event: QEnterEvent) -> None:
    self.isHovered = True
    self.update()

  def leaveEvent(self, event: QEnterEvent) -> None:
    self.isHovered = False
    self.update()
