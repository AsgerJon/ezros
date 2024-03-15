"""DataView provides a descriptor for instances of QChartView."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Self

from PySide6.QtCharts import QChartView
from PySide6.QtCore import QEvent
from vistutils.fields import unParseArgs, Wait, FloatField


class DataView(QChartView):
  """The DataView class provides a descriptor for instances of QChartView."""

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Creates a default instance"""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ViewField(Wait):
  """The ViewField class provides a descriptor for instances of
  QChartView."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ViewField."""
    Wait.__init__(self, DataView, *args, **kwargs)
