"""IntField provides an integer valued field"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
from typing import Any

from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField


class AbstractNumberField(AbstractBaseField):
  """IntField provides an integer valued field"""

  __default_value__ = None

  @classmethod
  @abstractmethod
  def _getFallbackValue(cls) -> Any:
    """Return the fallback value for the field."""

  @abstractmethod
  def getFieldType(self) -> type:
    """Return the type of the field."""

  @abstractmethod
  def getDefaultValue(self) -> Any:
    """Return the default value for the field."""

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter function for the field"""
    if instance is None:
      return self.getDefaultValue()
    pvtName = self.getPrivateName()
    if getattr(instance, pvtName, None) is not None:
      value = getattr(instance, pvtName)
      if isinstance(value, self.getFieldType()):
        return value
      e = typeMsg(value, self.getFieldType())
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self.getDefaultValue())
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any, **kwargs) -> None:
    """Setter function for the field"""
    pvtName = self.getPrivateName()
    if not isinstance(value, self.getFieldType()):
      try:
        if kwargs.get('_recursion', False):
          raise RecursionError
        reCasted = self.getFieldType()(value)
        return self.__set__(instance, reCasted, _recursion=True)
      except Exception as exception:
        e = typeMsg(value, self.getFieldType())
        raise exception from TypeError(e)
    setattr(instance, pvtName, value)
