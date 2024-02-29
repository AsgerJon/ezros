"""TextField provides a strongly typed descriptor containing text."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Optional

from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from morevistutils import Field, TypedField


class TextField(TypedField):
  """The TextField class provides a strongly typed descriptor containing
  text."""

  __default_value__ = None
  __fallback_value__ = ''

  def getFieldType(self, ) -> type:
    """Returns the field type."""
    return str

  def getDefaultValue(self) -> Any:
    """Returns the default value."""
    if self.__default_value__ is None:
      return self.__fallback_value__
    return self.__default_value__
