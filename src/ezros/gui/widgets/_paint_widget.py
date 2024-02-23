"""PaintWidget is the base visible widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPaintEvent, QColor

from ezros.gui.factories import solidBrush, emptyBrush, parsePen, emptyPen
from ezros.gui.widgets import BaseWidget


class PaintWidget(BaseWidget):
  """PaintWidget is the base visible widget. """

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)

  def paintHook(self, painter: QPainter, event: QPaintEvent) -> None:
    """Implementation of paint event"""

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event"""
    painter = QPainter()
    painter.begin(self)
    self.paintHook(painter, event)
    painter.end()
