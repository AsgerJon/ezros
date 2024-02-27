"""PaintWidget is the base visible widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import TYPE_CHECKING

from PySide6.QtGui import QPainter, QPaintEvent
from icecream import ic
from vistutils.waitaminute import typeMsg

from ezros.gui.widgets import BaseWidget

if TYPE_CHECKING:
  from ezros.gui.paint import AbstractPaint


class PaintWidget(BaseWidget):
  """PaintWidget is the base visible widget. """

  __painting_active__ = None
  __hooked_paints__ = None

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)

  def getWidget(self) -> PaintWidget:
    """Getter for the widget."""
    return self

  @classmethod
  def _getHookedPaints(cls, **kwargs) -> list[AbstractPaint]:
    """Getter for the hooked paints."""
    if getattr(cls, '__hooked_paints__', None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      setattr(cls, '__hooked_paints__', [])
      return cls._getHookedPaints(_recursion=True)
    if isinstance(cls.__hooked_paints__, list):
      return cls.__hooked_paints__
    e = typeMsg('hooked_paints', cls.__hooked_paints__, list)
    raise TypeError(e)

  @classmethod
  def appendHookedPaint(cls, paint: AbstractPaint) -> None:
    """Appends a hooked paint."""
    existing = cls._getHookedPaints()
    setattr(cls, '__hooked_paints__', [*existing, paint])

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""
    self._getHookedPaints()
    for paint in self.__hooked_paints__:
      paint.paintOp(event, painter)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    # painter.setRenderHint(QPainter.Antialiasing, True)
    self.__painting_active__ = True
    self.paintHook(event, painter)
    self.__painting_active__ = False
    painter.end()
