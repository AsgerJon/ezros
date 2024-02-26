"""DataPaint paints data points on the widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QPen, QColor
from vistutils.waitaminute import typeMsg
from PySide6.QtGui import QPainter, QPaintEvent

from ezros.gui.factories import emptyBrush
from ezros.gui.paint import AbstractPaint
from morevistutils import DataEcho


class DataPaint(AbstractPaint):
  """DataPaint paints data points on the widget"""

  __fallback_pen__ = None

  @classmethod
  def _createFallbackPen(cls) -> None:
    """Creates the fallback pen"""
    pen = QPen()
    pen.setWidth(4)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    pen.setColor(QColor(0, 0, 0))
    setattr(cls, '__fallback_pen__', pen)

  @classmethod
  def getFallbackPen(cls, **kwargs) -> QPen:
    """Returns the fallback pen"""
    if cls.__fallback_pen__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      cls._createFallbackPen()
      return cls.getFallbackPen(_recursion=True)
    if isinstance(cls.__fallback_pen__, QPen):
      return cls.__fallback_pen__
    e = typeMsg('__fallback_pen__', cls.__fallback_pen__, QPen)
    raise TypeError(e)

  def __init__(self, *args) -> None:
    n, self.pen = None, None
    for arg in args:
      if isinstance(arg, int) and n is None:
        n = arg
      elif isinstance(arg, QPen) and self.pen is None:
        self.pen = arg
    self.dataEcho = DataEcho(n)
    self.pen = self.getFallbackPen() if self.pen is None else self.pen

  def paintOp(self, event: QPaintEvent, painter: QPainter) -> None:
    """Applies the paint operation"""
    painter.setPen(self.pen)
    painter.setBrush(emptyBrush())
    for point in self.dataEcho @ event.rect():
      painter.drawPoint(point)
