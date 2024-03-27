"""Button implementation"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, Qt, QMargins
from PySide6.QtGui import QMouseEvent, \
  QPaintEvent, \
  QPainter, \
  QPen, \
  QBrush, \
  QColor, QFont
from PySide6.QtWidgets import QApplication
from ezside.core import Black, Center
from ezside.widgets import BaseWidget, TextLabel
from icecream import ic

from ezros.defaults import Settings


class Button(TextLabel):
  """Button implementation"""
  __mouse_down__ = False
  __under_mouse__ = False
  __click_armed__ = False
  __checked_state__ = False
  __enabled_state__ = True

  singleClick = Signal()

  def mousePressEvent(self, event: QMouseEvent) -> None:
    """Handle mouse press events."""
    if self.__mouse_down__:
      return
    self.__mouse_down__ = True
    self.__click_armed__ = True
    self.update()

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Handle mouse release events."""
    if self.__click_armed__:
      self.singleClick.emit()
    self.__click_armed__ = False
    self.__mouse_down__ = False
    self.update()

  def enterEvent(self, event: QMouseEvent) -> None:
    """Handle mouse enter events."""
    self.__under_mouse__ = True
    self.update()

  def leaveEvent(self, event: QMouseEvent) -> None:
    """Handle mouse leave events."""
    self.__under_mouse__ = False
    self.__click_armed__ = False
    self.__mouse_down__ = False
    self.update()

  def __init__(self, *args, ) -> None:
    """Initialize the widget."""
    TextLabel.__init__(self, *args, )
    self.setMouseTracking(True)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMouseTracking(True)

  def getBorderRadius(self) -> int:
    """Return the border radius."""
    radius = 4
    if QApplication.mouseButtons():
      radius += 12
    if self.__under_mouse__:
      radius += 4
    return radius

  def getBorderWidth(self) -> int:
    """Return the border width."""
    width = 1
    if QApplication.mouseButtons():
      width += 7
    if self.__under_mouse__:
      width += 2
    return width

  def getBorderPen(self) -> QPen:
    """Return the border pen that depends on the state."""
    width = self.getBorderWidth()
    radius = self.getBorderRadius()
    color = Black
    pen = QPen()
    pen.setColor(color)
    pen.setWidth(width)
    pen.setStyle(Qt.PenStyle.SolidLine)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    return pen

  def getBackgroundFill(self) -> QColor:
    """Return the background fill."""
    r, g, b, a = 255, 191, 63, 255
    if QApplication.mouseButtons():
      g -= 63
      b -= 31
    if self.__under_mouse__:
      g -= 63
      b -= 31
    return QColor(r, g, b, a)

  def getBackgroundBrush(self) -> QBrush:
    """Return the background brush."""
    brush = QBrush()
    brush.setColor(self.getBackgroundFill())
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    return brush

  def getBorderMargins(self) -> QMargins:
    """Return the border margins."""
    if self.__under_mouse__:
      return QMargins(4, 4, 4, 4)
    return QMargins(3, 3, 3, 3, )

  def getFontSize(self) -> int:
    """Return the font size."""
    size = 12
    if QApplication.mouseButtons():
      size += 1
    if self.__under_mouse__:
      size += 1
    return size

  def getFont(self) -> QFont:
    """Return the font."""
    font = QFont()
    font.setPointSize(16)
    return font

  def getTextBackgroundBrush(self) -> QBrush:
    """Return the text background brush."""
    brush = QBrush()
    alpha = 31
    if self.__under_mouse__:
      alpha += 31
    if QApplication.mouseButtons():
      alpha += 63
    brush.setColor(QColor(255, 255, 255, alpha))
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    return brush

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the widget."""
    painter = QPainter()
    painter.begin(self)
    radius = self.getBorderRadius()
    pen = self.getBorderPen()
    brush = self.getBackgroundBrush()
    viewRect = self.rect()
    paddedRect = viewRect.marginsRemoved(self.getBorderMargins())
    painter.setPen(pen)
    painter.setBrush(brush)
    painter.drawRoundedRect(paddedRect, radius, radius)
    painter.setPen(self.emptyLine)
    painter.setBrush(self.getTextBackgroundBrush())
    text = self.getText()
    font = self.getFont()
    painter.setFont(font)
    painter.setPen(self.fontLine)
    tightText = painter.boundingRect(viewRect, Center, text)
    textRect = tightText.marginsAdded(QMargins(8, 8, 8, 8, ))
    textRadius = int(radius // 2)
    painter.drawRoundedRect(textRect, textRadius, textRadius)
    painter.setBrush(self.emptyBrush)
    painter.setPen(self.fontLine)
    painter.drawText(tightText, text)
    painter.end()
