"""IntField provides a number valued descriptor class of type int."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.parse import maybe

from morevistutils.fields import AbstractNumberField


class IntField(AbstractNumberField):
  """IntField provides a number valued descriptor class of type int."""

  __default_value__ = None

  @classmethod
  def _getFallbackValue(cls) -> int:
    """Return the fallback value for the field."""
    return 0

  def getFieldType(self) -> type:
    """Return the type of the field."""
    return int

  def getDefaultValue(self) -> int:
    """Return the default value for the field."""
    return maybe(self.__default_value__, self._getFallbackValue(), )
