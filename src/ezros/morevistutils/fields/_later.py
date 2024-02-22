"""Later provides fields with delayed instantiation. They require as
argument a callable and are allowed to provide further arguments later
used to invoke the callable. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any, Self

from icecream import ic
from vistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class Later:
  """Later provides fields with delayed instantiation. They require as
  argument a callable and are allowed to provide further arguments later
  used to invoke the callable. """

  def __init__(self, callMeMaybe: Callable, *args, **kwargs) -> None:
    if isinstance(callMeMaybe, type):
      def creator(*args2, **kwargs2) -> Any:
        """Inferred creator function"""
        return callMeMaybe(*args2, **kwargs2)

      self.__creator_function__ = creator
    elif isinstance(callMeMaybe, Callable):
      self.__creator_function__ = callMeMaybe
    elif callMeMaybe is not None:
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__field_name__ = None
    self.__field_owner__ = None
    self.__creator_function__ = callMeMaybe

    self.__positional_args__ = args
    self.__keyword_args__ = kwargs

  def __set_name__(self, owner: type, name: str) -> None:
    """Setter function for the field name and owner."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateName(self, ) -> str:
    """Getter function for the private name"""
    return '_%s' % self.__field_name__

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter function for the field"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.createObject(instance, owner)
    return self.__get__(instance, owner, _recursion=True)

  def createObject(self, instance: Any, owner: type) -> None:
    """This method creates the object expected at the dot. """
    if self.__creator_function__ is None:
      e = """The field was accessed before creator function were set!"""
      raise AttributeError(e)
    if not isinstance(self.__creator_function__, Callable):
      e = typeMsg('creator function', self.__creator_function__, Callable)
      raise TypeError(e)
    creator = self.__creator_function__
    args = [instance, owner, *self.__positional_args__]
    kwargs = self.__keyword_args__
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, creator(*args, **kwargs))

  def _setCreator(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for the creator function"""
    if self.__creator_function__ is not None:
      e = """Creator function already set!"""
      raise AttributeError(e)
    if not isinstance(callMeMaybe, Callable):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__creator_function__ = callMeMaybe
    return callMeMaybe

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Alias for _setCreator. Use this as decorator."""
    return self._setCreator(callMeMaybe)

  def __rshift__(self, callMeMaybe: Callable) -> Self:
    """Alias for _setCreator except returning self"""
    self._setCreator(callMeMaybe)
    return self

  def __matmul__(self, callMeMaybe: Callable) -> Self:
    """Alias for _setCreator except returning self"""
    self._setCreator(callMeMaybe)
    return self
