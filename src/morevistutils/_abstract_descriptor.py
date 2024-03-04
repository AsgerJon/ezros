"""AbstractDescriptor provides an abstract baseclass for descriptors."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from icecream import ic
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from morevistutils import MetaDescriptor


class AbstractDescriptor:
  """AbstractDescriptor provides an abstract baseclass for descriptors."""

  __field_name__: str = None
  __field_owner__: type = None
  __field_type__: type = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Invoked automatically after the owner is created, but before the
    owner is initiated. Let Owner be a class derived from a custom
    metaclass Meta, let Field be a descriptor derived from type and let
    field be an instance of Field owned by owner. Then the following
    ordering of events occur:
    1.  namespace = Meta.__prepare__(mcls, name, bases, ) creates the
        namespace object
    2.  field = Field() creates the field instance
    3   Owner = type.__new__(mcls, name, bases, namespace) creates the
        owner class
    4.  field.__set_name__(Owner, name) sets the field name and owner
    5.  type.__init__(Owner, name, bases, namespace) initiates the owner
        class"""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getFieldType(self) -> type:
    """Getter-function for getting the field type."""
    if self.__field_type__ is None:
      e = """Field type not defined!"""
      raise AttributeError(e)
    if isinstance(self.__field_type__, type):
      return self.__field_type__
    e = typeMsg('__field_type__', self.__field_type__, type)
    raise TypeError(e)

  def _setFieldType(self, fieldType: type) -> None:
    """Setter-function for field type"""
    if self.__field_type__ is not None:
      e = """Field type already defined!"""
      raise AttributeError(e)
    if not isinstance(fieldType, type):
      e = typeMsg('fieldType', fieldType, type)
      raise TypeError(e)
    self.__field_type__ = fieldType

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

  def _instantiate(self, instance: object, owner: type = None) -> Any:
    """This method is invoked when the instance does not have a value on
    the private field name. This default implementation invokes this
    method, before retrying the getter operation. Subclasses may
    reimplement this method to provide a default value. By default,
    this method raises the expected AttributeError."""
    try:
      object.__getattribute__(instance, self._getPrivateName())
    except AttributeError as attributeError:
      e = ("""This descriptor class does not provide a default value,
      but was accessed on instance: '%s' owned by: '%s' without an
      explicitly set value!""" % (instance, owner))
      raise NotImplementedError(monoSpace(e)) from attributeError

  def _typeGuard(self, value: object) -> Any:
    """Compares the given value with the type expected."""
    fieldType = self._getFieldType()
    if isinstance(value, fieldType):
      return value
    e = typeMsg('value', value, fieldType)
    raise TypeError(e)

  def __get__(self, instance: object, owner: type, **kwargs) -> Any:
    """Getter function"""
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    ic(pvtName)
    if getattr(instance, pvtName, None) is not None:
      val = getattr(instance, pvtName)
      return self._typeGuard(val)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self._instantiate(instance, owner)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: object, value: object) -> None:
    """Setter function"""
    pvtName = self._getPrivateName()
    setattr(self, pvtName, self._typeGuard(value))

  def __delete__(self, instance: object) -> None:
    """Deleter function"""
    pvtName = self._getPrivateName()
    if hasattr(self, pvtName):
      return delattr(self, pvtName)
    e = """The instance: '%s' has no attribute at given name: '%s'!"""
    raise AttributeError(monoSpace(e % (instance, pvtName)))
