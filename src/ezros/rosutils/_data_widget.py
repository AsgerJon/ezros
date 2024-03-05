"""DataWidget provides data plotting on a widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Slot, QMargins, QRect
from PySide6.QtGui import QPainter, QPaintEvent
from icecream import ic

from ezros.gui.paint import BorderRect, FillRect, DataPaint
from ezros.gui.widgets import PaintWidget

ic.configureOutput(includeContext=True)


class DataWidget(PaintWidget):
  """DataWidget provides data plotting on a widget"""

  fillPaint = FillRect()
  borderPaint = BorderRect()
  dataPaint = DataPaint()

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)

  @Slot(float)
  def callback(self, data: Any) -> None:
    """Callback for data update"""
    self.dataPaint.dataEcho.append(data.data)

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""
    viewRect = painter.viewport()
    bottomMargin = 32
    innerRect = viewRect - QMargins(64, 32, 32, 32)
    hTickWidth = viewRect.width()
    hTickHeight = 24
    hTickTop = viewRect.bottom() + bottomMargin / 2 - hTickHeight / 2
    hTickLeft = viewRect.left()
    hTickRect = QRect(hTickLeft, hTickTop, hTickWidth, hTickHeight)
    dataEvent = QPaintEvent(innerRect)
    self.fillPaint.paintOp(event, painter)
    self.borderPaint.paintOp(event, painter)
    self.dataPaint.paintOp(dataEvent, painter)

  def getRect(self) -> QRect:
    """Return the rectangle of the widget."""
    return self.geometry()

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the widget."""
    painter = QPainter(self)
    viewRect = painter.viewport()

    painter.end()
    event.accept()
