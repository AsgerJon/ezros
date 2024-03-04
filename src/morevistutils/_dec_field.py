"""DecField provides a descriptor class requiring accessor functions to
be set explicitly using decorators. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.waitaminute import typeMsg


class DecField:
  """The DecField class provides a descriptor class requiring accessor
  functions to be set explicitly using decorators."""

  __field_name__ = None
  __field_owner__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getFieldName(self) -> str:
    """Getter-function for getting the field name."""
    if self.__field_name__ is None:
      e = """Field name not defined!"""
      raise AttributeError(e)
    if isinstance(self.__field_name__, str):
      return self.__field_name__
    e = typeMsg('__field_name__', self.__field_name__, str)
    raise TypeError(e)

  def _getPrivateName(self) -> str:
    """Getter-function for getting the private name."""
    return '_%s' % self._getFieldName()

  def _getFieldOwner(self) -> type:
    """Getter-function for getting the field owner."""
    if self.__field_owner__ is None:
      e = """Field owner not defined!"""
      raise AttributeError(e)
    if isinstance(self.__field_owner__, type):
      return self.__field_owner__
    e = typeMsg('__field_owner__', self.__field_owner__, type)
    raise TypeError(e)
