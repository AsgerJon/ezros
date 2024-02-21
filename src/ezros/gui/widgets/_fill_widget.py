"""FillWidget fills the background of the widget with solid color"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPaintEvent, QColor

from ezros.gui.factories import solidBrush, emptyBrush, parsePen, emptyPen
from ezros.gui.widgets import BaseWidget


class FillWidget(BaseWidget):
  """FillWidget fills the background of the widget with solid color"""

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event"""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    fillBrush = solidBrush(QColor(127, 255, 0, 255))
    blankBrush = emptyBrush()
    borderColor = QColor(0, 0, 0, 255)
    borderPen = parsePen(borderColor, 1, Qt.PenStyle.SolidLine)
    blankPen = emptyPen()
    painter.setBrush(fillBrush)
    painter.setPen(blankPen)
    painter.drawRect(viewRect)
    painter.setBrush(blankBrush)
    painter.setPen(borderPen)
    painter.drawRect(viewRect)
    painter.end()
