"""ButtonField provides a descriptor field for Qt.MouseButton enums."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.shortnames import MouseBtn
from _dep.morevistutils import TypedField


class ButtonField(TypedField):
  """The ButtonField class provides a strongly typed descriptor containing
  Qt.MouseButton enums."""

  __default_value__ = None
  __fallback_value__ = None

  def getFieldType(self, ) -> type:
    """Returns the field type."""
    return MouseBtn

  def getDefaultValue(self) -> MouseBtn:
    """Returns the default value."""
    if self.__default_value__ is None:
      return self.__fallback_value__
    return self.__default_value__
