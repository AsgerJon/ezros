"""AbstractPaint provides the abstract base class for the paint hooks."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Self

from PySide6.QtGui import QPaintEvent, QPainter
from vistutils.text import monoSpace

from ezros.gui.widgets import PaintWidget


class AbstractPaint:
  """AbstractPaint provides the abstract base class for the paint hooks."""

  __active_widget__ = None
  __field_name__ = None
  __field_owner__ = None

  def __init__(self, *args, **kwargs) -> None:
    """If you never implement __init__ and you allow the instances to use
    object.__init__, you will not be able to pass any arguments up through
    the chain of classes. This is by intention of course, for some reason,
    but during development it is stupid. Instead, have at least a method
    absorbing the arguments. This way, later on you can include arguments
    in the inheritance chain without having to change a bunch of code."""

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the name of the attribute"""
    if issubclass(owner, PaintWidget):
      self.__field_name__ = name
      self.__field_owner__ = owner
      return owner.appendHookedPaint(self)
    e = """Only subclasses of PaintWidget may own descriptors of the 
    AbstractPaint class and subclasses! Class: '%s' is not a subclass 
    of PaintWidget.""" % owner.__qualname__
    raise RuntimeError(monoSpace(e))

  def getFieldName(self) -> str:
    """Returns the name of the field"""
    return self.__field_name__

  def getFieldOwner(self) -> type:
    """Returns the owner of the field"""
    return self.__field_owner__

  def getWidget(self, ) -> Self:
    """Returns the widget"""
    return self.__active_widget__

  def setWidget(self, widget: Self) -> None:
    """Sets the widget"""
    self.__active_widget__ = widget

  def delWidget(self, ) -> None:
    """Deletes the widget"""
    self.__active_widget__ = None

  def applyPaint(self, event: QPaintEvent, painter: QPainter) -> None:
    """Paints the background"""
    if self.getWidget() is not painter.device():
      e = """The widget is not the device of the painter!"""
      raise RuntimeError(e)
