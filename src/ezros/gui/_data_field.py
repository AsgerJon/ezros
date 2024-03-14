"""DataField provides a descriptor on the QXYSeries class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Self, Any

from PySide6.QtCharts import QXYSeries, QScatterSeries, QLineSeries
from PySide6.QtCore import QPointF, Slot
from vistutils.fields import unParseArgs, Wait


class DataSeries(QXYSeries):
  """The DataSeries class provides a strongly typed descriptor containing
  QXYSeries objects."""

  @staticmethod
  def rightNow() -> float:
    """Returns the current time."""
    return time.time()

  @Slot(float)
  def add(self, value: float) -> None:
    """Appends the given value to the series."""
    self.append(value)

  def append(self, *args, ) -> None:
    """Appends the given points to the series."""
    if len(args) == 1:
      if isinstance(args[0], (float, int,)):
        return self.append(QPointF(self.rightNow(), args[0]))
    return QXYSeries.append(self, *args)

  def getScatter(self) -> QScatterSeries:
    """Returns the scatter series."""
    scatterSeries = QScatterSeries()
    scatterSeries.append(self.points())
    return scatterSeries

  def getLine(self) -> QLineSeries:
    """Returns the line series."""
    lineSeries = QLineSeries()
    lineSeries.append(self.points())
    return lineSeries

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Creates a default instance"""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class DataField(Wait):
  """The DataField class provides a descriptor for instances of
  DataSeries."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the DataField."""
    Wait.__init__(self, DataSeries, *args, **kwargs)
