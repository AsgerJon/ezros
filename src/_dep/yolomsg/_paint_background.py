"""PaintBackground fills the viewport of the widget with solid color"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QRect
from PySide6.QtGui import QPainter, QPaintEvent

from _dep.yolomsg._abstract_paint_rect import AbstractPaintRect


class PaintBackground(AbstractPaintRect):
  """PaintBackground fills the viewport of the widget with solid color"""

  def getRect(self, event: QPaintEvent, painter: QPainter) -> QRect:
    """Get the rectangle to paint"""
    return painter.viewport()
