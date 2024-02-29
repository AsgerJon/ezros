"""Wait provides a descriptor with a callable that is used to create the
object at the dot. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from icecream import ic
from vistutils.waitaminute import typeMsg


class Wait:
  """Wait provides a descriptor with a callable that is used to create the
  object at the dot. """

  __creator_function__ = None
  __field_name__ = None
  __field_owner__ = None
  __field_type__ = None
  __positional_args__ = None
  __keyword_args__ = None
  __include_owner__ = None
  __include_instance__ = None

  @staticmethod
  def fromType(cls: type) -> Callable:
    """Creates a creator function from a type"""

    def callMeMaybe(*args, **kwargs) -> Any:
      """Creates an object from a type"""
      return cls(*args, **kwargs)

    return callMeMaybe

  def __init__(self, *args, **kwargs) -> None:
    self.__creator_function__ = None
    self.__positional_args__ = []
    for arg in args:
      if isinstance(arg, type) and self.__creator_function__ is None:
        self.__creator_function__ = self.fromType(arg)
      elif callable(arg) and self.__creator_function__ is None:
        self.__creator_function__ = arg
      else:
        self.__positional_args__.append(arg)
    self.__keyword_args__ = {}
    self.__include_owner__ = None
    self.__include_instance__ = None
    ownerKeys = ['owner', 'includeOwner', 'include_owner']
    instanceKeys = ['instance', 'includeInstance', 'include_instance']
    self.__include_owner__ = None
    self.__include_instance__ = None
    for (key, val) in kwargs.items():
      if key in ownerKeys and self.__include_owner__ is None:
        self.__include_owner__ = val
      elif key in instanceKeys and self.__include_instance__ is None:
        self.__include_instance__ = val
      else:
        self.__keyword_args__[key] = val
    if self.__include_owner__ is None:
      self.__include_owner__ = True
    if self.__include_instance__ is None:
      self.__include_instance__ = True

  def __set_name__(self, owner: type, name: str) -> None:
    """Automatically invoked my the type metaclass after __new__,
    but before __init__."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def __rrshift__(self, other: type) -> Wait:
    """Right shift operator for the field type"""
    self.__field_type__ = other
    return self

  def _getFieldType(self) -> type:
    """Getter-function for the field type"""
    return object if self.__field_type__ is None else self.__field_type__

  def _getPrivateName(self) -> str:
    """Getter-function for the private name"""
    return '_%s' % self.__field_name__

  def _getExisting(self, instance, ) -> Any:
    """Getter-function for the existing object"""
    pvtName = self._getPrivateName()
    if getattr(instance, pvtName, None) is None:
      raise AttributeError(pvtName)
    item = getattr(instance, pvtName)
    if isinstance(item, self._getFieldType()):
      return item
    e = typeMsg('item', item, self._getFieldType())
    raise TypeError(e)

  def _createObject(self, instance: Any, owner: type) -> None:
    """Creates the instance"""
    if self.__creator_function__ is None:
      e = """The field was accessed before creator function were set!"""
      raise AttributeError(e)
    args = [*self.__positional_args__]
    if self.__include_owner__:
      args = [owner, *args]
    if self.__include_instance__:
      args = [instance, *args]
    kwargs = {**self.__keyword_args__, }
    item = None
    try:
      item = self.__creator_function__(*args, **kwargs)
    except TypeError as typeError:
      if 'required positional argument' in str(typeError):
        raise typeError
      if 'takes' in str(typeError) and 'positional' in str(typeError):
        try:
          item = self.__creator_function__(instance, *args, **kwargs)
        except Exception as exception:
          raise exception from typeError
    if item is None:
      e = """Failure during object creation!"""
      raise RuntimeError(e)
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, item)

  def __get__(self, instance, owner, **kwargs) -> Any:
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    try:
      return self._getExisting(instance, )
    except AttributeError as attributeError:
      if kwargs.get('_recursion', False):
        raise RecursionError from attributeError
      self._createObject(instance, owner)
      return self.__get__(instance, owner, _recursion=True)

  def _setCreator(self, callMeMaybe: Callable) -> Callable:
    """Sets the creator"""
    if hasattr(callMeMaybe, '__func__'):
      self.__creator_function__ = callMeMaybe.__func__
    else:
      self.__creator_function__ = callMeMaybe
    return callMeMaybe

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Decorator for the creator"""
    return self._setCreator(callMeMaybe)
