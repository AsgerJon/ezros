"""ArrayField provides a multidimensional number valued descriptor class
of fixed shape and type. Please note that the terminology used in this
class were first VectorField, but to avoid confusion with the vector
valued space in linear algebra, the name was changed to ArrayField."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.text import monoSpace

Shape = tuple[int, ...]
NumType = type[int | float | complex]


class ArrayField:
  """ArrayField provides a multidimensional number valued descriptor class
  of fixed shape and type. Please note that the terminology used in this
  class were first VectorField, but to avoid confusion with the vector
  valued space in linear algebra, the name was changed to ArrayField."""

  __default_value__ = None

  def __init__(self, type_: NumType, shp: Shape, *args, **kwargs) -> None:
    """Initialize the ArrayField instance."""
    if 0 in shp:
      raise ValueError('ArrayField shape cannot contain 0')
    if any([dim < 0 for dim in shp]):
      raise ValueError('ArrayField shape cannot contain negative values')
    shape = [*sorted([*shp, ], ), ]
    self.__vector_length__ = shape.pop()
    self.__vector_type__ = type_
    if any([dim != 1 for dim in shape]):
      e = """Only one dimensional arrays are supported at the moment."""
      raise NotImplementedError(e)

  def _getFallbackValue(self) -> tuple[NumType]:
    """Return the fallback value for the field."""
    value = {int: 0, float: .0, complex: 0j, }[self.__vector_type__]
    return (*[value for _ in range(self.__vector_length__)],)

  def _getDefaultValue(self) -> tuple[NumType]:
    """Return the default value for the field."""
    return self._getFallbackValue()

  def __getattribute__(self, key: str) -> Any:
    """Please note that this method was reimplemented by an experienced
    professional under controlled conditions. DO NOT TRY THIS AT HOME!"""
    if key == '__default_value__':
      cls = object.__getattribute__(self, '__class__')
      name = object.__getattribute__(cls, '__name__')
      e = """__default_value__ is not an attribute of '%s'!""" % name
      raise AttributeError(monoSpace(e))
    return object.__getattribute__(self, key)
