"""FlexField provides a descriptor class for mutable types. When first
accessed on an instance, an object of the mutable type is created and
assigned to the instance. The setter and deleter will then augment or
reset the object as appropriate."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from icecream import ic
from vistutils.waitaminute import typeMsg

from morevistutils import AbstractDescriptor

ic.configureOutput(includeContext=True)


class FlexField(AbstractDescriptor):
  """FlexField provides a descriptor class for mutable types. When first
  accessed on an instance, an object of the mutable type is created and
  assigned to the instance. The setter and deleter will then augment or
  reset the object as appropriate."""

  __explicit_creator__: Callable = None
  __positional_args__: list = None
  __keyword_args__: dict = None

  def CREATOR(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the creator method for the field. """
    return self._setCreator(callMeMaybe)

  def _setCreator(self, callMeMaybe: Callable) -> Callable:
    """Sets the creator method for the field. """
    if self.__explicit_creator__ is not None:
      e = """Creator method already defined!"""
      raise AttributeError(e)
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__explicit_creator__ = getattr(callMeMaybe, '__func__', callMeMaybe)
    return callMeMaybe

  def _getCreator(self, ) -> Callable:
    """Getter for the creator method for the field. """
    if self.__explicit_creator__ is None:
      e = """Creator method not defined!"""
      raise AttributeError(e)
    if not callable(self.__explicit_creator__):
      e = typeMsg('__explicit_creator__',
                  self.__explicit_creator__,
                  Callable)
      raise TypeError(e)
    return self.__explicit_creator__

  def __init__(self, *args, **kwargs) -> None:
    for arg in args:
      if callable(arg) and self.__explicit_creator__ is None:
        self._setCreator(arg)
        break
      if isinstance(arg, type) and self.__field_type__ is None:
        creator = getattr(arg, 'getDefault', None)
        if creator is not None:
          if callable(creator):
            self._setCreator(creator)
            self._setFieldType(arg)
            break
    self.__positional_args__ = [*args, ]
    self.__keyword_args__ = kwargs

  def _getFieldType(self) -> type:
    """Getter-function for field type"""
    if self.__field_type__ is None:
      return object
    return AbstractDescriptor._getFieldType(self)

  def _instantiate(self, instance: object, owner: type = None) -> Any:
    """Instantiates the field on the instance. """
    creator = self._getCreator()
    args = [instance, self._getPrivateArgs(instance)]
    kwargs = self._getPrivateKwargs(instance)
    pvtName = self._getPrivateName()
    obj = creator(*args, **kwargs)
    ic(pvtName)
    setattr(instance, pvtName, obj)

  def __class_getitem__(cls: type, key: type) -> type:
    """Class method for getting the field type. """
    name = cls.__name__
    bases = (cls,)
    namespace = {'__field_type__': key, }
    return type(name, (cls,), namespace)

  def _getPrivateArgsName(self, ) -> str:
    """Getter-function for getting the private args name."""
    return '__%s_args__' % self._getFieldName()

  def _getPrivateArgs(self, instance: object) -> list:
    """Getter-function for getting the private args."""
    return getattr(instance, self._getPrivateArgsName(), [])

  def _getPrivateKwargsName(self, ) -> str:
    """Getter-function for getting the private kwargs name."""
    return '__%s_kwargs__' % self._getFieldName()

  def _getPrivateKwargs(self, instance: object) -> dict:
    """Getter-function for getting the private kwargs."""
    return getattr(instance, self._getPrivateKwargsName(), {})

  def _getFieldArgs(self) -> list:
    """Getter-function for getting positional arguments included when
    instantiating the field."""
    return self.__positional_args__

  def _getFieldKwargs(self) -> dict:
    """Getter-function for getting keyword arguments included when
    instantiating the field."""
    return self.__keyword_args__

  def _getArgs(self, instance: object) -> list:
    """Getter-function for getting the args."""
    return [*self._getPrivateArgs(instance), *self._getFieldArgs(), ]

  def _getKwargs(self, instance: object) -> dict:
    """Getter-function for getting the kwargs."""
    return {**self._getPrivateKwargs(instance), **self._getFieldKwargs(), }

  def __set__(self, instance: object, value: Any) -> None:
    """Setter function"""
    args, kwargs = None, None
    if isinstance(value, (tuple, list)):
      if len(value) != 2:
        e = """Tuple or list must contain exactly two elements!"""
        raise ValueError(e)
      if not isinstance(value[1], dict):
        e = """Unable to parse"""
        raise ValueError(e)
      kwargs = value[1]
      if isinstance(value[0], (tuple, list)):
        args = [*value[0], ]
      else:
        args = [value[0], ]
    setattr(instance, self._getPrivateArgsName(), args)
    setattr(instance, self._getPrivateKwargsName(), kwargs)
