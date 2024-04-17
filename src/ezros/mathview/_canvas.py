"""The Canvas class provides the widget actually rendering the mathematical
object under observation. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

import numpy as np
from PySide6.QtCore import QPointF, QMarginsF
from PySide6.QtGui import QPaintEvent, \
  QPainter, \
  QPen, \
  QColor, \
  QBrush, \
  QPolygonF
from ezside.core import SolidLine, BlankFill, SolidFill
from ezside.widgets import BaseWidget
from vistutils.waitaminute import typeMsg


class Canvas(BaseWidget):
  """The Canvas class provides the widget actually rendering the mathematical
  object under observation. """
  __real_to_real__ = None
  __fallback_num_points__ = 256
  __horizontal_min__ = -1
  __horizontal_max__ = 1
  __vertical_min__ = None
  __vertical_max__ = None

  @staticmethod
  def compile(mappingStr: str) -> Callable:
    """Compile the mapping."""
    func = eval(f'lambda x: {mappingStr}')

    def wrapper(x: float) -> float:
      """Wrapper for the function."""
      x += 0j
      out = func(x)
      if out.imag ** 2 > 1e-06:
        return 0.
      return out.real

    return wrapper

  def setMapping(self, *args) -> None:
    """Set the mapping of the canvas."""
    callableCode = None
    for arg in args:
      if callable(arg):
        self.__real_to_real__ = arg
        break
      if isinstance(arg, str) and callableCode is None:
        callableCode = arg
    else:
      self.__real_to_real__ = self.compile(callableCode)
    self.update()

  def getMapping(self, ) -> Callable:
    """Return the mapping of the canvas."""
    if self.__real_to_real__ is None:
      return lambda x: (1 + x * x) ** 0.5
    if callable(self.__real_to_real__):
      return self.__real_to_real__
    e = typeMsg('mapping', self.__real_to_real__, Callable)
    raise TypeError(e)

  def setRange(self, min_: float, max_: float) -> None:
    """Set the range of the canvas."""
    self.__vertical_min__ = min_
    self.__vertical_max__ = max_

  def getRange(self) -> tuple[float, float]:
    """Return the range of the canvas."""
    return self.__vertical_min__, self.__vertical_max__

  def setDomain(self, min_: float, max_: float) -> None:
    """Set the domain of the canvas."""
    self.__horizontal_min__ = min_
    self.__horizontal_max__ = max_

  def getDomain(self) -> tuple[float, float]:
    """Return the domain of the canvas."""
    return self.__horizontal_min__, self.__horizontal_max__

  def getPoints(self) -> tuple[list[float], list[float]]:
    """Return the points of the canvas."""
    x0, x1, n = [*self.getDomain(), self.__fallback_num_points__][:3]
    X = np.linspace(x0, x1, n, endpoint=True, dtype=np.float32)
    Y = np.array([self.getMapping()(x) for x in X], dtype=np.float32)
    X = [x.real if isinstance(x, complex) else x for x in X]
    xMin, xSpan = min(X), max(X) - min(X)
    X = [x - xMin for x in X]
    X = [x / xSpan for x in X]
    yMin, ySpan = min(Y), max(Y) - min(Y)
    Y = [y - yMin for y in Y]
    Y = [y / ySpan for y in Y]
    Y = [y.real if isinstance(y, complex) else y for y in Y]
    return X, Y

  def paintEvent(self, event: QPaintEvent) -> None:
    """The paintEvent method paints the canvas."""
    painter = QPainter()
    painter.begin(self, )
    viewRect = painter.viewport().toRectF()
    borderRect = viewRect - QMarginsF(2, 2, 2, 2, )
    mathRect = viewRect - QMarginsF(10, 10, 10, 10, )
    borderRect.moveCenter(viewRect.center())
    pixelWidth, pixelHeight = mathRect.width(), mathRect.height()
    #  Border rectangle outline
    borderPen = QPen()
    borderPen.setColor(QColor(63, 63, 63, 255))
    borderPen.setWidth(1)
    borderPen.setStyle(SolidLine)
    borderBrush = QBrush()
    borderBrush.setColor(QColor(0, 0, 0, 0, ))
    borderBrush.setStyle(BlankFill)
    painter.setPen(borderPen)
    painter.setBrush(borderBrush)
    painter.drawRect(borderRect)
    # # # # # #
    #  Placing each point
    pointPen = QPen()
    pointPen.setColor(QColor(0, 0, 0, 255))
    pointPen.setWidth(1)
    pointPen.setStyle(SolidLine)
    pointBrush = QBrush()
    pointBrush.setColor(QColor(0, 0, 63, 255))
    pointBrush.setStyle(SolidFill)
    painter.setPen(pointPen)
    painter.setBrush(pointBrush)
    points = []
    for (x, y) in zip(*self.getPoints()):
      xp, yp = x * pixelWidth + 10, y * pixelHeight + 10
      points.append(QPointF(xp, yp))
    polygon = QPolygonF.fromList([*points, *points[::1]])
    painter.drawPolygon(polygon)
    painter.end()

  def initUi(self) -> None:
    """Initialize the user interface of the canvas."""
    self.setMinimumSize(256, 256)

  def connectActions(self) -> None:
    """Connect the actions of the canvas."""
    pass
