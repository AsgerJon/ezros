"""MetaDescriptor provides a custom metaclass used to create descriptors."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from vistutils.metas import AbstractMetaclass
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg


class FieldType:
  """FieldType provides a singleton descriptor on the descriptor types."""

  __field_name__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner."""
    self.__field_name__ = name

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

  def __get__(self, cls: type, mcls: type) -> Any:
    """Returns the class type."""
    if cls is None:
      return self
    fieldType = getattr(cls, '__field_type__', None)
    if fieldType is None:
      e = """Class '%s' derived from '%s' does not implement a field type, 
      which is required for instance checking!""" % (
        cls.__name__, cls.__qualname__)
      raise AttributeError(monoSpace(e))
    if isinstance(fieldType, type):
      return fieldType
    e = typeMsg('__field_type__', fieldType, type)
    raise TypeError(e)

  def __set__(self, cls: type, fieldType: type) -> None:
    """Sets the field type."""
    pvtName = self._getPrivateName()
    if getattr(cls, pvtName, None) is not None:
      e = """Field type already defined!"""
      raise AttributeError(monoSpace(e))
    if not isinstance(fieldType, type):
      e = typeMsg('fieldType', fieldType, type)
      raise TypeError(e)
    setattr(cls, pvtName, fieldType)

  def __delete__(self, *_) -> Never:
    """Illegal deleter function"""
    raise TypeError("""FieldType is a protected class!""")


class FieldName:
  """FieldName provides a singleton descriptor on the descriptor names."""

  __pvt_name__ = '__field_name__'

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner."""
    if name != 'fieldName':
      e = """Instances of FieldName may only be named 'fieldName', 
      but received: '%s'!""" % name
      raise NameError(monoSpace(e))

  def __get__(self, instance: object, owner: type) -> Any:
    """Returns the field name."""
    if instance is None:
      return self
    name = getattr(instance, self.__pvt_name__, None)
    if name is None:
      e = """Field name not defined!"""
      raise AttributeError(e)
    if isinstance(name, str):
      return name
    e = typeMsg('__field_name__', name, str)
    raise TypeError(e)


class MetaDescriptor(AbstractMetaclass):
  """MetaDescriptor provides a custom metaclass used to create
  descriptors."""

  fieldName = FieldName()
  fieldOwner = FieldType()
  fieldType = FieldType()

  def __instancecheck__(cls, instance: object) -> bool:
    """Considers the field type as the instance type. """

  def __rrshift__(cls, other: object) -> bool:
    """Considers the field type as the instance type. """
    return cls.__instancecheck__(other)

  def __eq__(cls, other: type) -> bool:
    """Considers the field type as the instance type. """
    if isinstance(other, type):
      if issubclass(other, cls.fieldType):
        return True
      if issubclass(cls.fieldType, other):
        return True
      return False
    return NotImplemented
