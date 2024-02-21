"""TypedField provides a strongly typed descriptor class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable, Never

from vistutils.fields import AbstractField, Field
from vistutils.parse import maybe
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg


class TypedField(AbstractField):
  """TypedField provides a strongly typed descriptor class"""

  __pre_init_hooks__ = []
  __post_init_hooks__ = []
  __owner_init__ = {}
  __owner_new__ = {}

  fieldOwner = Field()

  @fieldOwner.GET
  def _getFieldOwner(self) -> type:
    """Getter-function for field owner"""
    if self.__field_owner__ is not None:
      if isinstance(self.__field_owner__, type):
        return self.__field_owner__
      e = typeMsg('__field_owner__', self.__field_owner__, type)
      raise TypeError(e)
    raise RuntimeError

  @fieldOwner.SET
  def _setFieldOwner(self, *_) -> Never:
    """Illegal setter function"""
    e = """The field owner is a read-only property."""
    raise AttributeError(e)

  @fieldOwner.DEL
  def _delFieldOwner(self) -> Never:
    """Illegal deleter function"""
    e = """The field owner is a read-only property."""
    raise AttributeError(e)

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(*args, **kwargs)
    parsed = [*args, None, None, None][:3]
    typeArg, defValArg, supportInitArg = parsed
    typeKwarg = kwargs.get('type', None)
    defValKwarg = kwargs.get('default', None)
    supportInitKwarg = kwargs.get('supportInit', None)
    if typeKwarg is None and typeArg is None:
      e = """The TypedField constructor requires a type argument or a type 
      keyword argument."""
      raise TypeError(e)
    valType = maybe(typeKwarg, typeArg)
    if not isinstance(valType, type):
      e = typeMsg('valType', valType, type)
      raise TypeError(e)
    self._valueType = valType
    defVal = maybe(defValKwarg, defValArg)
    if defVal is not None:
      if not isinstance(defVal, valType):
        e = """The default value must be of the same type as the value 
        type."""
        raise TypeError(monoSpace(e))
    else:
      try:
        defVal = valType()
      except Exception as exception:
        e = """Failed to create a appropriate default value for the given 
        type: %s""" % self._valueType
        raise ValueError(e) from exception
    self._defVal = defVal
    self._supportInit = maybe(supportInitKwarg, supportInitArg, False)

  def _getInitPreHooks(self) -> list[Callable]:
    """Getter-function for init pre hooks"""
    return self.__pre_init_hooks__

  def hookPreInit(self, callMeMaybe: Callable) -> Callable:
    """Decorator for adding pre init hooks"""
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self._getInitPreHooks().append(callMeMaybe)
    return callMeMaybe

  def _updateInit(self) -> None:
    """Updates the __init__"""

    def hookedInit(this: Any, *args, **kwargs) -> None:
      """Replacement initiator"""
      for hook in self._getInitPreHooks():
        hook(this, *args, **kwargs)
      self.__owner_init__(this, *args, **kwargs)
      for hook in self._getInitPostHooks():
        hook(this, *args, **kwargs)

    setattr(self.fieldOwner, '__init__', hookedInit)

  def _getInitPostHooks(self) -> list[Callable]:
    """Getter-function for init post hooks"""
    return self.__post_init_hooks__

  def hookPostInit(self, callMeMaybe: Callable) -> Callable:
    """Decorator for adding post init hooks"""
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self._getInitPostHooks().append(callMeMaybe)
    return callMeMaybe

  @classmethod
  def __prepare_owner__(cls, owner: type) -> None:
    """This special abstract method must be implemented by subclasses to
    install this field into it."""
    cls.__owner_init__ |= {owner: getattr(owner, '__init__')}
    cls.__owner_new__ |= {owner: getattr(owner, '__new__')}
