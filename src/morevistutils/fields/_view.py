"""View is a read only descriptor class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any, Never

from vistutils.waitaminute import typeMsg

from morevistutils.fields import Primitive


class View(Primitive):
  """View is a read only descriptor class. """

  __explicit_getter__ = None

  def _setExplicitGetter(self, callMeMaybe: Callable) -> None:
    """This method is called when the descriptor is called. """
    self.__explicit_getter__ = callMeMaybe

  def __call__(self, callMeMaybe: Callable) -> Callable:
    """This method is called when the descriptor is called. """
    self._setExplicitGetter(callMeMaybe)
    return callMeMaybe

  def __init__(self, callMeMaybe: Callable, **kwargs) -> None:
    pass

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter for the field"""
    if self.__explicit_getter__ is None:
      return Primitive.__get__(self, instance, owner, **kwargs)
    callMeMaybe = self.__explicit_getter__
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    return callMeMaybe(instance, )

  def __set__(self, *_) -> Never:
    """Illegal setter function"""
    e = """The field is a read-only property."""
    raise AttributeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal deleter function"""
    e = """The field is a read-only property."""
    raise AttributeError(e)
