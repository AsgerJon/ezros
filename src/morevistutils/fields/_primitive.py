"""Primitive provides a simple descriptor class for primitive uses."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.waitaminute import typeMsg


class Primitive:
  """Simple property class"""

  __field_name__ = None
  __field_owner__ = None
  __private_name__ = None
  __default_value__ = None
  __value_type__ = None

  def __init__(self, **kwargs) -> None:
    self.__value_type__ = kwargs.get('type', None)
    self.__default_value__ = kwargs.get('default', None)
    if self.__value_type__ is None:
      if self.__default_value__ is not None:
        self.__value_type__ = type(self.__default_value__)
      else:
        e = """No arguments received!"""
        raise ValueError(e)

  def __set_name__(self, owner: type, name: str) -> None:
    self.__field_name__ = name
    self.__field_owner__ = owner
    self.__private_name__ = '_%s' % name

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    if instance is None:
      return self.__get__(owner, owner)
    if hasattr(instance, self.__private_name__):
      return getattr(instance, self.__private_name__)
    kwargs.get('_recursion', False)
    setattr(instance, self.__private_name__, self.__default_value__)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: Any, value: Any) -> None:
    if isinstance(value, self.__value_type__):
      return setattr(instance, self.__private_name__, value)
    e = typeMsg('value', value, self.__value_type__)
    raise TypeError(e)
