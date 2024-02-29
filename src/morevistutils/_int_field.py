"""IntField provides a strongly typed descriptor field for integers"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from morevistutils import TypedField


class IntField(TypedField):
  """The IntField class provides a strongly typed descriptor containing
  integers."""

  __default_value__ = None
  __fallback_value__ = 0

  def getFieldType(self, ) -> type:
    """Returns the field type."""
    return int

  def getDefaultValue(self) -> Any:
    """Returns the default value."""
    if self.__default_value__ is None:
      return self.__fallback_value__
    return self.__default_value__
