"""PointPlot provides a widget for plotting points"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

import numpy as np
from PySide6.QtCore import QPoint, QPointF, QSize, QRect
from PySide6.QtGui import QPaintEvent, QPainter
from icecream import ic
from vistutils.parse import maybe

from ezros.gui.factories import parsePen, parseBrush, emptyPen
from ezros.gui.shortnames import Black, SolidLine, Yellow, Spread
from ezros.gui.widgets import FillWidget

ic.configureOutput(includeContext=True)


class PointPlot(FillWidget):
  """PointPlot provides a widget for plotting points"""

  leftMargin = 96
  rightMargin = 16
  topMargin = 32
  bottomMargin = 32

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)
    self.setSizePolicy(Spread, Spread)
    self._numPoints = 128
    self._data = np.zeros((self._numPoints,), dtype=complex)
    self._firstIndex = 0
    self._minValue = 0
    self._minValueAge = 0
    self._maxValue = 0
    self._maxValueAge = 0
    t = 0
    for i in range(self._numPoints * 2):
      self.append(np.sin(t))
      t += np.pi / 24

  def getTimeRange(self, ) -> tuple[float, float]:
    """Getter for timeRange."""
    return self._data.real.min(), self._data.real.max()

  def getValueRange(self, ) -> tuple[float, float]:
    """Getter for valueRange."""
    return self._minValue, self._maxValue

  def append(self, value: float = None) -> None:
    """Append a value to the list of points"""
    self._append(value)

  def _append(self, value: float = None) -> None:
    """Append a value to the list of points"""
    newValue = maybe(value, 0.0)
    self._data[self._firstIndex] = time.time() + newValue * 1J
    self._firstIndex -= 1
    while self._firstIndex < 0:
      self._firstIndex += self._numPoints
    if self._data.real.min():
      medianVal = self._minValue / 2 + self._maxValue / 2
      timeSpan = self._data.real.max() - self._data.real.min()
      minRel = medianVal - self._minValue
      maxRel = self._maxValue - medianVal
      minDecay = 1 - np.tanh(self._minValueAge / timeSpan)
      maxDecay = 1 - np.tanh(self._maxValueAge / timeSpan)
      minTest = medianVal - minRel * minDecay
      maxTest = medianVal + maxRel * maxDecay
      if newValue < minTest:
        self._minValue = newValue
        self._minValueAge = 0
      if newValue > maxTest:
        self._maxValue = newValue
        self._maxValueAge = 0

  def _getData(self, ) -> tuple[np.ndarray, np.ndarray]:
    """Getter for data."""
    data = np.roll(self._data, -self._firstIndex)
    T = np.interp(data.real, (data.real.min(), data.real.max()), (0, 1))
    X = np.interp(data.imag, (self._minValue, self._maxValue), (0, 1))
    return T, X

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event"""
    painter = QPainter()
    painter.begin(self)

    viewRect = painter.viewport()
    viewSize = viewRect.size()
    viewWidth = viewSize.width()
    viewHeight = viewSize.height()
    plotWidth = viewWidth - self.leftMargin - self.rightMargin
    plotHeight = viewHeight - self.topMargin - self.bottomMargin
    plotTopLeft = QPoint(self.leftMargin, self.topMargin)
    plotSize = QSize(plotWidth, plotHeight)
    plotRect = QRect(plotTopLeft, plotSize)
    T, X = self._getData()  # Real values
    t, x = T * plotWidth + self.leftMargin, X * plotHeight + self.topMargin
    points = [QPointF(t, x) for t, x in zip(t, x)]
    painter.setBrush(parseBrush(Yellow))
    painter.setPen(emptyPen())
    painter.drawRect(plotRect)
    painter.setPen(parsePen(Black, 6, SolidLine))
    painter.drawPoints(points)
    painter.end()
