"""Creates a callable applying text to a paint widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPaintEvent, QColor, QFont
from PySide6.QtCore import QRect


def textPainter(painter: QPainter, event: QPaintEvent) -> None:
  """Applies text to the painter"""
  viewRect = painter.viewport()
