"""Wait provides a descriptor class for deferred instance creation."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.fields import CoreDescriptor
from vistutils.waitaminute import typeMsg


class Wait(CoreDescriptor):
  """A descriptor class for deferred instance creation."""

  __field_class__ = None
  __default_creator__ = None
  __default_value__ = None

  def __init__(self, defaultArg: Any = None, *args, **kwargs) -> None:
    """Initializes the descriptor."""
    CoreDescriptor.__init__(self, *args, **kwargs)
    if isinstance(defaultArg, type):
      self._setFieldClass(defaultArg)
    elif callable(defaultArg):
      self._setDefaultCreator(defaultArg)
    else:
      self._setDefaultValue(defaultArg)

  def _setFieldClass(self, cls: type) -> type:
    """Setter-function for the field class."""
    if isinstance(cls, type):
      if getattr(cls, 'getDefault', None) is None:
        e = """The class must have a method named getDefault."""
        raise AttributeError(e)
      getDefault = getattr(cls, 'getDefault')
      if not callable(getDefault):
        e = typeMsg('getDefault', getDefault, Callable)
        raise TypeError(e)
      self.__field_class__ = cls
      return self.__field_class__
    else:
      e = typeMsg('cls', cls, type)
      raise TypeError(e)

  def _getFieldClass(self, ) -> Any:
    """Getter-function for the field class."""
    if self.__field_class__ is None:
      e = """Field class not defined!"""
      raise AttributeError(e)
    return self.__field_class__

  def _setDefaultCreator(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for the default creator."""
    if callable(callMeMaybe):
      self.__default_creator__ = callMeMaybe
    else:
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    return callMeMaybe

  def _setDefaultValue(self, value: Any) -> None:
    """Setter-function for the default value."""
    self.__default_value__ = value
    return self.__default_value__

  def _getDefaultValue(self, instance: object) -> Any:
    """Getter-function for the default value."""
    if self.__default_value__ is not None:
      return self.__default_value__
    if self.__default_creator__ is not None:
      return self.__default_creator__(instance, )

  def _instantiate(self, instance: object) -> None:
    """Creates an object of the given field class for the instance given."""
    cls = self._getFieldClass()
    obj = cls.getDefault()
    setattr(instance, self._getPrivateName(), obj)

  def CREATE(self, callMeMaybe: Any) -> Any:
    """Assigns the creator"""
    if isinstance(callMeMaybe, type):
      return self._setFieldClass(callMeMaybe)
    if callable(callMeMaybe):
      return self._setDefaultCreator(callMeMaybe)
    return self._setDefaultValue(callMeMaybe)
