"""ChartField provides a descriptor for instances of QChart."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Self

from PySide6.QtCharts import QChart, QAbstractAxis
from PySide6.QtCore import QPointF, Qt
from vistutils.fields import unParseArgs, Wait


class DataChart(QChart):
  """The ChartField class provides a descriptor for instances of QChart."""

  def horizontalAxis(self, ) -> QAbstractAxis:
    """Returns the horizontal axis."""
    return self.axes(Qt.Orientation.Horizontal)[0]

  def verticalAxis(self, ) -> QAbstractAxis:
    """Returns the vertical axis."""
    return self.axes(Qt.Orientation.Vertical)[0]

  def horizontalPixel(self, value: float) -> float:
    """Returns the pixel position corresponding to the given value."""
    return self.mapToPosition(QPointF(value, 0)).x()

  def verticalPixel(self, value: float) -> float:
    """Returns the pixel position corresponding to the given value."""
    return self.mapToPosition(QPointF(0, value)).y()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Creates a default instance"""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ChartField(Wait):
  """The ChartField class provides a descriptor for instances of
  QChart."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ChartField."""
    Wait.__init__(self, DataChart, *args, **kwargs)
