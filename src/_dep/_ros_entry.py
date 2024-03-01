"""RosEntry provides a descriptor class for use in the Ros namespace."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Never, Any


class _TypeField:
  """Descriptor class for use in the Ros namespace."""

  __field_owner__ = None
  __field_type__ = None
  __type_name__ = None

  def __init__(self, fieldType: type) -> None:
    self.__field_type__ = fieldType

  def __set_name__(self, owner, name) -> None:
    self.__field_owner__ = owner

  def __get__(self, instance, owner) -> Any:
    return self.__field_type__


class RosEntry:
  """RosEntry provides a descriptor class for use in the Ros namespace."""

  __field_name__ = None
  __field_owner__ = None

  __field_type__ = None
  __ros_type__ = None
  __ros_array__ = None
  __ros_alts__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the name of the attribute"""
    __field_name__ = name
    __field_owner__ = owner

  def __init__(self, *args, **kwargs) -> None:
    self.__field_type__ = kwargs.get('fieldType', )
    self.__ros_type__ = kwargs.get('fieldType', )
    self.__ros_array__ = kwargs.get('rosArray', )
    self.__ros_alts__ = kwargs.get('rosAlts', )

  def __get__(self, instance: object, owner: type) -> object:
    """Gets the value of the attribute"""
    return self.__field_type__

  def __set__(self, instance: object, value: object) -> Never:
    """Illegal setter function"""
    e = """Illegal setter function"""
    raise TypeError(e)

  def __delete__(self, instance: object, ) -> Never:
    """Illegal deleter function"""
    e = """Illegal deleter function"""
    raise TypeError(e)
