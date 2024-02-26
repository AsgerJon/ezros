"""FlexField provides a descriptor class free to decorate any method as
the getter and setter functions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField


class FlexField(AbstractBaseField):
  """FlexField provides a descriptor class free to decorate any method as
  the getter and setter functions."""

  __explicit_getter__ = None
  __explicit_setter__ = None
  __explicit_deleter__ = None

  def __init__(self, *args, **kwargs) -> None:
    AbstractBaseField.__init__(self, *args, **kwargs)

  def __get__(self, instance: Any, owner: Any) -> Any:
    getter = self.getGetter()
    return getter(instance, owner)

  def __set__(self, instance: Any, value: Any) -> None:
    try:
      setter = self.getSetter()
    except AttributeError as attributeError:
      e = """The field named '%s' is read-only!""" % self.getPrivateName()
      raise TypeError(e) from attributeError
    setter(instance, value)

  def __delete__(self, instance: Any) -> None:
    try:
      deleter = self.getDeleter()
    except AttributeError as attributeError:
      e = """The field named '%s' is read-only!""" % self.getPrivateName()
      raise TypeError(e) from attributeError
    deleter(instance)

  def setGetter(self, func: Callable) -> Callable:
    """Decorate a method as the getter function."""
    if self.__explicit_getter__ is not None:
      e = 'Getter function already set.'
      raise AttributeError(e)
    if not callable(func):
      e = typeMsg('func', func, Callable)
      raise TypeError(e)
    self.__explicit_getter__ = func
    return func

  def getGetter(self, ) -> Callable:
    """Getter-function for the getter."""
    if self.__explicit_getter__ is None:
      defaultGetter = self.getDefaultGetter()
      if defaultGetter is not None:
        if callable(defaultGetter):
          return defaultGetter
        e = typeMsg('defaultGetter', defaultGetter, Callable)
        raise TypeError(e)
      e = 'Getter function not set.'
      raise AttributeError(e)
    if callable(self.__explicit_getter__):
      return self.__explicit_getter__
    e = typeMsg('getter', self.__explicit_getter__, Callable)
    raise TypeError(e)

  def setSetter(self, func: Callable) -> Callable:
    """Decorate a method as the setter function."""
    if self.__explicit_setter__ is not None:
      e = 'Setter function already set.'
      raise AttributeError(e)
    if not callable(func):
      e = typeMsg('func', func, Callable)
      raise TypeError(e)
    self.__explicit_setter__ = func
    return func

  def getSetter(self, ) -> Callable:
    """Getter-function for the setter."""
    if self.__explicit_setter__ is None:
      defaultSetter = self.getDefaultSetter()
      if defaultSetter is not None:
        if callable(defaultSetter):
          return defaultSetter
        e = typeMsg('defaultSetter', defaultSetter, Callable)
        raise TypeError(e)
      e = 'Setter function not set.'
      raise AttributeError(e)
    if callable(self.__explicit_setter__):
      return self.__explicit_setter__
    e = typeMsg('setter', self.__explicit_setter__, Callable)
    raise TypeError(e)

  def setDeleter(self, func: Callable) -> Callable:
    """Decorate a method as the deleter function."""
    if self.__explicit_deleter__ is not None:
      e = 'Deleter function already set.'
      raise AttributeError(e)
    if not callable(func):
      e = typeMsg('func', func, Callable)
      raise TypeError(e)
    self.__explicit_deleter__ = func
    return func

  def getDeleter(self, ) -> Callable:
    """Getter-function for the deleter."""
    if self.__explicit_deleter__ is None:
      defaultDeleter = self.getDefaultDeleter()
      if defaultDeleter is not None:
        if callable(defaultDeleter):
          return defaultDeleter
        e = typeMsg('defaultDeleter', defaultDeleter, Callable)
        raise TypeError(e)
      e = 'Deleter function not set.'
      raise AttributeError(e)
    if callable(self.__explicit_deleter__):
      return self.__explicit_deleter__
    e = typeMsg('deleter', self.__explicit_deleter__, Callable)
    raise TypeError(e)

  def getDefaultGetter(self, ) -> Callable:
    """This fallback may be reimplemented by subclasses. """

  def getDefaultSetter(self, ) -> Callable:
    """This fallback may be reimplemented by subclasses. """

  def getDefaultDeleter(self, ) -> Callable:
    """This fallback may be reimplemented by subclasses. """

  def SET(self, func: Callable) -> Callable:
    """Decorate a method as the setter function."""
    return self.setSetter(func)

  def GET(self, func: Callable) -> Callable:
    """Decorate a method as the getter function."""
    return self.setGetter(func)

  def DEL(self, func: Callable) -> Callable:
    """Decorate a method as the deleter function."""
    return self.setDeleter(func)
