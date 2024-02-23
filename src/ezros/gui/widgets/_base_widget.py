"""BaseWidget provides the lowest level of abstraction for a QWidget. It is
controls only size limits."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from PySide6.QtWidgets import QWidget, QSizePolicy

from ezros.gui.factories import dashPen

MinExp = QSizePolicy.Policy.MinimumExpanding
Max = QSizePolicy.Policy.Maximum


class BaseWidget(QWidget):
  """BaseWidget provides the lowest level of abstraction for a QWidget. It is
  controls only size limits."""

  paintingActive = Flag(False)

  def __init__(self, *args, **kwargs) -> None:
    for arg in args:
      if isinstance(arg, QWidget):
        QWidget.__init__(self, arg)
        break
    else:
      QWidget.__init__(self, )

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""

  def highlight(self, event: QPaintEvent, painter: QPainter) -> None:
    """Highlights the widget."""
    painter.setPen(dashPen())
    viewRect = painter.viewport()
    painter.drawRect(viewRect)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    self.paintHook(event, painter)

    painter.end()
