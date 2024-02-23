"""AbstractBaseField provides a baseclass for descriptor classes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg


class AbstractBaseField:
  """AbstractBaseField provides a baseclass for descriptor classes."""

  __owned_instances__ = {}
  __hooked_methods__ = {}

  def __init__(self, *args, **kwargs) -> None:
    self.__field_name__ = None
    self.__field_owner__ = None

  def __set_name__(self, owner, name) -> None:
    """This method sets the name and owner of the field and is
    automatically called when the owner class is created. """
    self.__field_name__ = name
    self.__field_owner__ = owner

  def __collect_pre_init_hooks__(self, ) -> list:
    """This method collects all preInit methods from the instance. """

  def __collect_post_init_hooks__(self, ) -> list:
    """This method collects all postInit methods from the instance. """

  @classmethod
  def getOwnedInstances(cls, owner: type) -> list:
    """This method returns a list of all instances of the descriptor
    class that are owned by the same owner class. """
    return cls.__owned_instances__.get(owner, [])

  @classmethod
  def preInit(cls, callMeMaybe: Callable) -> Callable:
    """Use this decorator to decorate methods that should be called before
    the owner. Please note, that these are shared by all instances of the
    descriptor class. This means that if an owner class has multiple
    instances of the descriptor class, methods decorated will be called
    only once. """

  def HOOK(self, target: str, interFace: Callable = None) -> Callable:
    """Methods decorated with this decorator, will apply 'interFace' to
    the callable named 'target' in the owner class. If 'decorated' is a
    method in the descriptor class, and 'original' is the method at the
    target name in the owner class, the following change will be applied
    to the owner class during '__set_name__':
    setattr(owner, target, interFace(decorated, original))
    The decorated method is returned. Please note, that a copy of the
    decorated class is passed to the owner class rather than a reference
    to the original. """

    def decorator(decorated: Callable) -> Callable:
      """This decorator returns the replacement method. """
      self.__hooked_methods__[target] = {'decorated': decorated,
                                         'interFace': interFace}
      return decorated

    return decorator

  def applyHooks(self, owner: type) -> None:
    """This method applies all hooks to the instance. """
    for target, hook in self.__hooked_methods__.items():
      decorated = hook['decorated']
      interFace = maybe(hook['interFace'], lambda *args: args[0])
      if hasattr(owner, target):
        original = getattr(owner, target)
        if callable(original):
          if hasattr(original, '__func__'):
            original = original.__func__
        else:
          e = typeMsg('original', original, Callable)
          raise TypeError(e)
      else:
        setattr(owner, target, decorated)
