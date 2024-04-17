"""MappingWidget visualizes a functional mapping showing the
transformation from the independent variables on the horizontal axis to
the dependent variables on the vertical axis. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.widgets import BaseWidget, EntryForm, Vertical

from ezros.mathview import Canvas


class MappingWidget(BaseWidget):
  """MappingWidget visualizes a functional mapping showing the
  transformation from the independent variables on the horizontal axis to
  the dependent variables on the vertical axis. """

  baseLayout = AttriBox[Vertical]()
  xMin = AttriBox[EntryForm]('xMin')
  xMax = AttriBox[EntryForm]('xMax')
  yMin = AttriBox[EntryForm]('yMin')
  yMax = AttriBox[EntryForm]('yMax')
  mapping = AttriBox[EntryForm]('Mapping')
  canvas = AttriBox[Canvas]()

  def range(self) -> tuple[float, float]:
    """Return the range of the mapping."""

  def domain(self) -> tuple[float, float]:
    """Return the domain of the mapping."""
