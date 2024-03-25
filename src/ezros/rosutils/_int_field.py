"""IntField with fixed init function"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg


class IntField:
  """IntField provides a strongly typed descriptor field for integers"""

  __field_name__ = None
  __field_owner__ = None
  __default_value__ = 0

  def __set_name__(self, owner, name) -> None:
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateFieldName(self) -> str:
    """Getter-function for the private field name"""
    return '_%s' % self.__field_name__

  def __get__(self, instance, owner, **kwargs) -> None:
    pvtName = self._getPrivateFieldName()
    if getattr(instance, pvtName, None) is None:
      setattr(instance, pvtName, self.__default_value__)
      return self.__get__(instance, owner, _recursion=True)
    return getattr(instance, pvtName)

  def __set__(self, instance, value) -> None:
    """Setter-function for the descriptor"""
    if isinstance(value, int):
      pvtName = self._getPrivateFieldName()
      return setattr(instance, pvtName, value)

  def __delete__(self, instance) -> None:
    """Deleter-function for the descriptor"""
    pvtName = self._getPrivateFieldName()
    if hasattr(instance, pvtName):
      return delattr(instance, pvtName)
    e = """No attribute found at name: '%s'""" % (pvtName,)
    raise AttributeError(monoSpace(e))

  def __init__(self, initValue: int = None) -> None:
    if initValue is None:
      self.__default_value__ = 0
    elif isinstance(initValue, int):
      self.__default_value__ = initValue
    elif isinstance(initValue, float):
      if initValue.is_integer():
        self.__default_value__ = int(initValue)
    else:
      e = typeMsg('initValue', initValue, int)
      raise TypeError(e)
