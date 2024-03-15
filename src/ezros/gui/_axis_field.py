"""AxisField provides a descriptor for instances of Axis."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Self

from PySide6.QtCharts import QValueAxis
from vistutils.fields import unParseArgs, Wait


class Axis(QValueAxis):
  """The Axis class provides a strongly typed descriptor containing
  QValueAxis objects."""

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Creates a default instance"""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class AxisField(Wait):
  """The AxisField class provides a descriptor for instances of
  Axis."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the AxisField."""
    Wait.__init__(self, Axis, *args, **kwargs)
