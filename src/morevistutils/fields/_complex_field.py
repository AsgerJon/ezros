"""ComplexField provides a number valued descriptor class of type complex."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.parse import maybe

from morevistutils.fields import AbstractNumberField


class ComplexField(AbstractNumberField):
  """ComplexField provides a number valued descriptor class of type
  complex."""

  __default_value__ = None

  @classmethod
  def _getFallbackValue(cls) -> complex:
    """Return the fallback value for the field."""
    return 0j

  def getFieldType(self) -> type:
    """Return the type of the field."""
    return complex

  def getDefaultValue(self) -> complex:
    """Return the default value for the field."""
    return maybe(self.__default_value__, self._getFallbackValue(), )
