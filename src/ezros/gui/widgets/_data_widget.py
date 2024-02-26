"""DataWidget provides data plotting on a widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Slot
from PySide6.QtGui import QPainter, QPaintEvent, QColor

from ezros.gui.factories import solidBrush, dashPen
from ezros.gui.paint import BorderRect, FillRect, DataPaint
from ezros.gui.shortnames import Lime
from ezros.gui.widgets import PaintWidget


class DataWidget(PaintWidget):
  """DataWidget provides data plotting on a widget"""

  def __init__(self, parent=None) -> None:
    super().__init__(parent)
    self.setMinimumSize(128, 128)
    self.fillPaint = FillRect(solidBrush(Lime))
    self.dataPaint = DataPaint()
    self.borderPaint = BorderRect(dashPen(QColor(127, 127, 127, 255)))

  def _getHookedPaints(self) -> list:
    """Returns the hooked paints"""
    return [self.fillPaint, self.dataPaint, self.borderPaint]

  @Slot(float)
  def callback(self, data: float) -> None:
    """Callback for data update"""
    self.dataPaint.dataEcho.append(float(data))

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint event"""
    painter = QPainter()
    painter.begin(self)
    self.fillPaint.paintOp(event, painter)
    self.dataPaint.paintOp(event, painter)
    self.borderPaint.paintOp(event, painter)
    painter.end()
