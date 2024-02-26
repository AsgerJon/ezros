"""ColorField provides a QColor valued descriptor class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QColor
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.gui.factories import parseColor
from ezros.gui.shortnames import White
from morevistutils.fields import AbstractBaseField


class ColorField(AbstractBaseField):
  """ColorField provides a QColor valued descriptor class."""

  __fallback_color__ = White
  __default_color__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the ColorField instance."""
    AbstractBaseField.__init__(self, *args, **kwargs)
    parsedColor = parseColor(*args, **kwargs, strict=False)
    self.__default_color__ = maybe(parsedColor, self.__fallback_color__, )

  def __get__(self, instance: Any, owner: type, **kwargs) -> QColor:
    """Getter function for the field"""
    if instance is None:
      return self.__default_color__
    pvtName = self.getPrivateName()
    if getattr(instance, pvtName, None) is not None:
      color = getattr(instance, pvtName)
      if isinstance(color, QColor):
        return color
      e = typeMsg('color', color, QColor)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self.__default_color__)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: QColor) -> None:
    """Setter function for the field"""
    pvtName = self.getPrivateName()
    if not isinstance(value, QColor):
      e = typeMsg('value', value, QColor)
      raise TypeError(e)
    setattr(instance, pvtName, value)
