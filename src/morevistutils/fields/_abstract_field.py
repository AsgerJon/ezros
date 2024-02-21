"""AbstractField provides an abstract baseclass for descriptor classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.waitaminute import typeMsg
from morevistutils.fields import Primitive


class AbstractField:
  """AbstractField provides an abstract baseclass for descriptor classes."""

  __all_owners__ = []
  __owner_fields__ = {}
  __owner_init__ = {}
  __pre_init_hooks__ = {}
  __post_init_hooks__ = {}

  @classmethod
  def _newOwner(cls, owner: type) -> None:
    """Registers a new owner class."""
    cls.__all_owners__.append(owner)
    cls.__owner_fields__[owner] = []
    cls.__owner_init__[owner] = getattr(owner, '__init__', None)

  @classmethod
  def _getBaseInit(cls, owner) -> Callable:
    """Getter-function for the base init method"""
    __base_init__ = getattr(owner, '__init__', )
    return __base_init__

  @classmethod
  def _getPreHooks(cls, owner: type) -> list[Callable]:
    """Getter-function for the pre-hooks"""
    return cls.__pre_init_hooks__.get(owner, [])

  @classmethod
  def _getPostHooks(cls, owner: type) -> list[Callable]:
    """Getter-function for the post-hooks"""
    return cls.__post_init_hooks__.get(owner, [])

  @classmethod
  def _appendHook(cls, owner: type, *args, **kwargs) -> None:
    """Registers a pre-hook for the init method"""
    if owner not in cls.__all_owners__:
      e = """The owner class is not registered as an owner class."""
      raise ValueError(e)
    callArgs = [arg for arg in args if callable(arg)]
    for callMeMaybe in callArgs:
      if not callable(callMeMaybe):
        e = typeMsg('callMeMaybe', callMeMaybe, Callable)
        raise TypeError(e)
      if kwargs.get('__pre__', False):
        existingHooks = cls._getPreHooks(owner)
        cls.__pre_init_hooks__[owner] = [*existingHooks, callMeMaybe]
      else:
        existingHooks = cls._getPostHooks(owner)
        cls.__post_init_hooks__[owner] = [*existingHooks, callMeMaybe]

  @classmethod
  def _appendPreHookInit(cls, owner: type, *args, **kwargs) -> None:
    """Registers a pre-hook for the init method"""
    cls._appendHook(owner, *args, __pre=True)
    cls._updateOwner(owner)

  @classmethod
  def _appendPostHookInit(cls, owner: type, *args, **kwargs) -> None:
    """Registers a post-hook for the init method"""
    cls._appendHook(owner, *args, __pre=False)
    cls._updateOwner(owner)

  @classmethod
  def _updateOwner(cls, owner) -> None:
    """Getter-function for the hooked init method"""
    preHooks, postHooks = cls.__pre_init_hooks__, cls.__post_init_hooks__
    __base_init__ = cls._getBaseInit(owner)

    def __hooked_init__(this: Any, *args, **kwargs) -> None:
      """The hooked init method"""
      if not isinstance(this, owner):
        e = """The hooked init method is called with an instance that is not 
        an instance of the owner class."""
        raise TypeError(e)
      for hook in preHooks:
        hook(this, *args, **kwargs)
      __base_init__(this, *args, **kwargs)
      for hook in postHooks:
        hook(this, *args, **kwargs)

    setattr(owner, '__init__', __hooked_init__)

  fieldOwner = Primitive(type=type, default=object)
  fieldName = Primitive(type=str, default='__un_named__')

  def __set_name__(self, owner: type, name: str) -> None:
    self._newOwner(owner)
    self.fieldOwner = owner
    self.fieldName = name
