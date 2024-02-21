"""Field class provides a descriptor for fields of custom classes. The
inner classes of the fields must be instantiated on receiving the field
owner and an instance hereof. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg


class Field:
  """Field class provides a descriptor for fields of custom classes. The
  inner classes of the fields must be instantiated on receiving the field
  owner and an instance hereof. """

  __global_field_registry__ = {}

  @classmethod
  def __class_getitem__(cls, key: str) -> Field:
    """Allows for field instances to be found by __getitem__ on the class
    itself."""
    if key in cls._getGlobalRegistry():
      return cls._getGlobalRegistry().get(key)
    raise KeyError(key)

  @classmethod
  def _getGlobalRegistry(cls) -> dict:
    """Getter-function for global registry"""
    return cls.__global_field_registry__

  def registerField(self, ) -> bool:
    """Registers field in global registry"""
    uniqueId = '__%s_%d__' % (self.__field_name__, hash(self))
    if uniqueId in self._getGlobalRegistry():
      raise AttributeError

  def __init__(self, cls: type, *args, **kwargs) -> None:
    self.__field_owner__ = None
    self.__field_name__ = None
    self.__field_type__ = cls
    self.__positional_args__ = args
    self.__keyword_args__ = kwargs

  def __set_name__(self, owner: type, name: str) -> None:
    self.__field_owner__ = owner
    self.__field_name__ = name
    if hasattr(owner, 'fieldHook'):
      owner.fieldHook(self)
    existingFields = getattr(owner, 'fieldRegistry', {})
    existing = existingFields.get(self.__field_type__, [])
    existingFields[self.__field_type__] = [*existing, self]
    setattr(owner, 'fieldRegistry', existingFields)

  def _getPrivateName(self) -> str:
    if self.__field_name__ is None:
      e = """Field instance has not been assigned to a class. """
      raise AttributeError(e)
    return '_%s' % self.__field_name__

  def _create(self, owner: type, instance: Any) -> None:
    """Creates and returns the object the instance will return. """
    args, kwargs = self.__positional_args__, self.__keyword_args__
    creator = None
    if hasattr(self, '__explicit_create__'):
      creator = getattr(self, '__explicit_create__')
    creator = self.__field_type__ if creator is None else creator
    if hasattr(creator, '__func__'):
      creator = getattr(creator, '__func__')
    if callable(creator):
      obj = creator(instance, owner, *args, **kwargs)
      if isinstance(obj, self.__field_type__):
        return obj
      e = typeMsg('value', obj, self.__field_type__)
      raise TypeError(e)
    e = typeMsg('creator', creator, Callable)
    raise TypeError(e)

  def _instantiate(self, owner: type, instance: Any) -> None:
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, self._create(owner, instance))

  def __get__(self, instance: Any, owner: Any, **kwargs) -> Any:
    if instance is None:
      return self.__field_type__
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      val = getattr(instance, pvtName)
      if isinstance(val, self.__field_type__):
        return val
      e = typeMsg('value', val, self.__field_type__)
      raise TypeError
    if kwargs.get('_recursion', False):
      raise RecursionError
    self._instantiate(owner, instance)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any) -> None:
    """Must be defined on the inner class. """
    raise NotImplementedError

  def __delete__(self, instance: Any) -> None:
    """Must be defined on the inner class. """
    raise NotImplementedError

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Decorator for the inner class of a field. """
    setattr(self, '__explicit_create__', callMeMaybe)
    return callMeMaybe
