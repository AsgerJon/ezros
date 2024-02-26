"""PaintWidget is the base visible widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPainter, QPaintEvent
from typing import TYPE_CHECKING

from vistutils.parse import maybe
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.gui.widgets import BaseWidget
from morevistutils.fields import Flag, TextField

if TYPE_CHECKING:
  from ezros.gui.widgets import AbstractPaint


class PaintWidget(BaseWidget):
  """PaintWidget is the base visible widget. """

  __painting_active__ = None

  paintingActive = Flag(False)
  innerText = TextField('')

  __hooked_paints__ = None

  @classmethod
  def _getHookedPaints(cls) -> list[AbstractPaint]:
    """Returns the hooked paints"""
    return getattr(cls, '__hooked_paints__', [])

  @classmethod
  def appendHookedPaint(cls, paint: AbstractPaint) -> None:
    """Appends a hooked paint"""
    hookName = '__hooked_paints__'
    existing = maybe(getattr(cls, hookName, []), [])
    if existing is None:
      e = """The 'maybe' function failed to find the empty list in 
      preference over 'None'!"""
      raise ValueError(e)
    if not isinstance(existing, list):
      e = typeMsg('existing', existing, list)
      raise TypeError(e)
    if paint in existing:
      e = """The paint '%s' is already hooked by class: '%s'!"""
      paintName = str(paint)
      clsName = cls.__qualname__
      raise RuntimeError(monoSpace(e % (paintName, clsName)))
    setattr(cls, hookName, [*existing, paint])

  @paintingActive.GET
  def getPaintingActive(self, ) -> bool:
    """Getter-function for the paintingActive flag."""
    return self.__painting_active__

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)

  def getWidget(self) -> PaintWidget:
    """Getter for the widget."""
    return self

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""
    for hook in self._getHookedPaints():
      hook.applyPaint(event, painter)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    self.__painting_active__ = True
    self.paintHook(event, painter)
    self.__painting_active__ = False
    painter.end()
