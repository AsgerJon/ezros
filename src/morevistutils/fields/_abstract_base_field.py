"""AbstractBaseField provides a baseclass for descriptor classes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.waitaminute import typeMsg


class AbstractBaseField:
  """AbstractBaseField provides a baseclass for descriptor classes."""

  def __init__(self, *args, **kwargs) -> None:
    self.__field_name__ = None
    self.__field_owner__ = None
    self.__field_type__ = None

  def getFieldType(self) -> type:
    """Getter function for the field type"""
    if self.__field_type__ is None:
      return object
    if isinstance(self.__field_type__, type):
      return self.__field_type__
    e = typeMsg('field_type', self.__field_type__, type)
    raise TypeError(e)

  def setFieldType(self, type_: type) -> None:
    """Setter function for the field type"""
    if type_ is None:
      return
    if not isinstance(type_, type):
      e = typeMsg('type_', type_, type)
      raise TypeError(e)
    if self.__field_type__ is not None:
      e = """The field type has already been set!"""
      raise AttributeError(e)
    self.__field_type__ = type_

  def __set_name__(self, owner, name) -> None:
    """This method sets the name and owner of the field and is
    automatically called when the owner class is created. """
    self.__field_name__ = name
    self.__field_owner__ = owner

  def getPrivateName(self, ) -> str:
    """Getter function for the private name"""
    return '_%s' % self.__field_name__

  def getFieldName(self, ) -> str:
    """Getter function for the field name"""
    return self.__field_name__

  def getFieldOwner(self, ) -> type:
    """Getter function for the field owner"""
    return self.__field_owner__

  def __get__(self, instance: Any, owner: type) -> Any:
    """Get the value of the field."""
    pvtName = self.getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    e = """The field named: '%s' does not exist in the instance: '%s'!"""
    raise AttributeError(e % (self.__field_name__, instance))

  def __set__(self, instance: Any, value: Any) -> None:
    """Set the value of the field."""
    pvtName = self.getPrivateName()
    setattr(instance, pvtName, value)

  def __delete__(self, instance: Any) -> None:
    """Delete the field."""
    pvtName = self.getPrivateName()
    if hasattr(instance, pvtName):
      return delattr(instance, pvtName)
    e = """The field named: '%s' does not exist in the instance: '%s'!"""
    raise AttributeError(e % (self.__field_name__, instance))
