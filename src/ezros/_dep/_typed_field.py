"""TypedField provides a baseclass for strongly typed descriptors"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
from typing import Any

from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from _dep.morevistutils import Field


class TypedField(Field):
  """The TypedField class provides a baseclass for strongly typed descriptors
  that can be used to create fields on classes that are strongly typed."""

  @abstractmethod
  def getFieldType(self) -> type:
    """Returns the field type."""

  @abstractmethod
  def getDefaultValue(self) -> Any:
    """Returns the default value."""

  def __get__(self, instance: object, owner: type, **kwargs) -> Any:
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if getattr(instance, pvtName, None) is not None:
      value = getattr(instance, pvtName)
      if isinstance(value, self.getFieldType()):
        return value
      e = typeMsg('value', value, self.getFieldType())
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self.getFieldType()())
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: object, value: Any) -> None:
    """Sets the value of the field."""
    if not isinstance(value, self.getFieldType()):
      e = typeMsg('value', value, self.getFieldType())
      raise TypeError(e)
    setattr(instance, self._getPrivateName(), value)

  def __delete__(self, instance: object) -> None:
    """Deletes the value"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return delattr(instance, pvtName)
    e = """The instance: '%s' has no attribute at given name: '%s'!"""
    raise AttributeError(monoSpace(e % (instance, pvtName)))
