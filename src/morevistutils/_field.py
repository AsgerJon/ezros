"""Field uses explicit setters and getters"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any, Never

from vistutils.waitaminute import typeMsg


class Field:
  """Field uses explicit setters and getters"""

  __field_name__ = None
  __field_owner__ = None

  __explicit_getter__ = None
  __explicit_setter__ = None
  __explicit_deleter__ = None

  def __init__(self, *args, **kwargs) -> None:
    pass

  def __set_name__(self, owner: type, name: str) -> None:
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getFieldName(self) -> str:
    """Getter-function for getting the field name."""
    if self.__field_name__ is None:
      e = """Field name not defined!"""
      raise AttributeError(e)
    if isinstance(self.__field_name__, str):
      return self.__field_name__
    e = typeMsg('__field_name__', self.__field_name__, str)
    raise TypeError(e)

  def _getFieldOwner(self) -> type:
    """Getter-function for getting the field owner."""
    if self.__field_owner__ is None:
      e = """Field owner not defined!"""
      raise AttributeError(e)
    if isinstance(self.__field_owner__, type):
      return self.__field_owner__
    e = typeMsg('__field_owner__', self.__field_owner__, type)
    raise TypeError(e)

  def fallbackGet(self, instance: object) -> Never:
    """This method is invoked when __get__ is called, but _getGetter does
    not return a getter. Subclasses may reimplement this method to provide
    fallback behavior."""
    fieldName = self._getFieldName()
    ownerName = self._getFieldOwner().__qualname__
    e = ("""Field: '%s' owned by '%s' has no getter, but an attempt was 
    made to get the value!""" % (fieldName, ownerName))
    raise AttributeError(e)

  def fallbackSet(self, instance: object, value: Any) -> Never:
    """This method is invoked when __set__ is called, but _getSetter does
    not return a setter. Subclasses may reimplement this method to provide
    fallback behavior."""
    fieldName = self._getFieldName()
    ownerName = self._getFieldOwner().__qualname__
    valueStr = str(value)
    e = """Field: '%s' owned by '%s' has no setter, but an attempt was 
    made to set the value to: '%s'!""" % (fieldName, ownerName, valueStr)
    raise AttributeError(e)

  def fallbackDel(self, instance: object) -> Never:
    """This method is invoked when __delete__ is called, but _getDeleter does
    not return a deleter. Subclasses may reimplement this method to provide
    fallback behavior."""
    fieldName = self._getFieldName()
    ownerName = self._getFieldOwner().__qualname__
    e = ("""Field: '%s' owned by '%s' has no deleter, but deletion was 
    attempted!""" % (fieldName, ownerName))
    raise AttributeError(e)

  def __get__(self, instance: object, owner: type) -> Any:
    """Getter for the field"""
    if instance is None:
      return self
    try:
      return self._getGetter()(instance)
    except TypeError as typeError:
      if 'Expected object at name' in str(typeError):
        return self.fallbackGet(instance)

  def __set__(self, instance: Any, value: Any) -> Any:
    """Setter for the field"""
    try:
      return self._getSetter()(instance, value)
    except TypeError as typeError:
      if 'Expected object at name' in str(typeError):
        return self.fallbackSet(instance, value)

  def __delete__(self, instance: Any) -> Any:
    """Deleter for the field"""
    try:
      return self._getDeleter()(instance)
    except TypeError as typeError:
      if 'Expected object at name' in str(typeError):
        return self.fallbackDel(instance)

  def _getGetter(self) -> Callable:
    """Getter-function for getting the getter."""
    if self.__explicit_getter__ is None:
      e = """No getter defined!"""
      raise AttributeError(e)
    if callable(self.__explicit_getter__):
      return self.__explicit_getter__
    e = typeMsg('__explicit_getter__', self.__explicit_getter__, Callable)
    raise TypeError(e)

  def _setGetter(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for setting the getter."""
    if self.__explicit_getter__ is not None:
      e = """Setter already defined!"""
      raise AttributeError(e)
    if callable(callMeMaybe):
      self.__explicit_getter__ = callMeMaybe
      return callMeMaybe
    e = typeMsg('callMeMaybe', callMeMaybe, Callable)
    raise TypeError(e)

  def _getSetter(self) -> Callable:
    """Getter-function for getting the setter."""
    if self.__explicit_setter__ is None:
      e = """No setter defined!"""
      raise AttributeError(e)
    if callable(self.__explicit_setter__):
      return self.__explicit_setter__
    e = typeMsg('__explicit_setter__', self.__explicit_setter__, Callable)
    raise TypeError(e)

  def _setSetter(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for setting the setter."""
    if self.__explicit_setter__ is not None:
      e = """Setter already defined!"""
      raise AttributeError(e)
    if callable(callMeMaybe):
      self.__explicit_setter__ = callMeMaybe
      return callMeMaybe
    e = typeMsg('callMeMaybe', callMeMaybe, Callable)
    raise TypeError(e)

  def _getDeleter(self) -> Callable:
    """Getter-function for getting the deleter."""
    if self.__explicit_deleter__ is None:
      e = """No deleter defined!"""
      raise AttributeError(e)
    if callable(self.__explicit_deleter__):
      return self.__explicit_deleter__
    e = typeMsg('__explicit_deleter__', self.__explicit_deleter__, Callable)
    raise TypeError(e)

  def _setDeleter(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for setting the deleter."""
    if self.__explicit_deleter__ is not None:
      e = """Deleter already defined!"""
      raise AttributeError(e)
    if callable(callMeMaybe):
      self.__explicit_deleter__ = callMeMaybe
      return callMeMaybe
    e = typeMsg('callMeMaybe', callMeMaybe, Callable)
    raise TypeError(e)

  def GET(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the getter."""
    return self._setGetter(callMeMaybe)

  def SET(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the setter."""
    return self._setSetter(callMeMaybe)

  def DELETE(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the deleter."""
    return self._setDeleter(callMeMaybe)
