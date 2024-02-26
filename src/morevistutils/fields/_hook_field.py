"""Abstract descriptor class for fields implementing hooks."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField
from morevistutils import Nunc


class HookField(AbstractBaseField):
  """Abstract descriptor class for fields implementing hooks."""

  __owned_instances__ = {}
  __hooked_methods__ = {}
  __registered_owners__ = []

  def __set_name__(self, owner, name) -> None:
    """This method sets the name and owner of the field and is
    automatically called when the owner class is created. """
    AbstractBaseField.__set_name__(self, owner, name)
    self.registerOwner(owner)
    self.applyHooks(owner)

  def registerOwner(self, owner: type) -> None:
    """This method registers the owner class. """
    if owner not in self.__registered_owners__:
      self.__registered_owners__.append(owner)
      self.__owned_instances__[owner] = []
    self.__owned_instances__[owner].append(self)

  @classmethod
  def getOwnedInstances(cls, owner: type) -> list:
    """This method returns a list of all instances of the descriptor
    class that are owned by the same owner class. """
    return cls.__owned_instances__.get(owner, [])

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

  def preSeq(self, target: str, ) -> Callable:
    """This hook runs the decorated method before running the original
    which then runs unaltered."""

    def interFace(decorated, original) -> Any:
      """This interface runs the decorated method before the original. """

      def wrapper(*args, **kwargs) -> Any:
        """This wrapper runs the decorated method before the original. """
        decorated(*args, **kwargs)
        return original(*args, **kwargs)

      return wrapper

    return self.HOOK(target, interFace)

  def postSeq(self, target: str, ) -> Callable:
    """This hook runs the decorated method after running the original
    which then runs unaltered."""

    def interFace(decorated, original) -> Any:
      """This interface runs the decorated method after the original. """

      def wrapper(*args, **kwargs) -> Any:
        """This wrapper runs the decorated method after the original. """
        result = original(*args, **kwargs)
        decorated(*args, **kwargs)
        return result

      return wrapper

    return self.HOOK(target, interFace)

  def preNest(self, target: str, ) -> Callable:
    """This hook runs the decorated method before running the original
    which then runs unaltered."""

    def interFace(decorated, original) -> Any:
      """This interface runs the decorated method before the original. """

      def wrapper(*args, **kwargs) -> Any:
        """This wrapper runs the decorated method before the original. """
        return decorated(original(*args, **kwargs), *args, **kwargs)

      return wrapper

    return self.HOOK(target, interFace)

  def postNest(self, target: str, ) -> Callable:
    """This hook runs the decorated method after running the original
    which then runs unaltered."""

    def interFace(decorated, original) -> Any:
      """This interface runs the decorated method after the original. """

      def wrapper(*args, **kwargs) -> Any:
        """This wrapper runs the decorated method after the original. """
        return original(decorated(*args, **kwargs), *args, **kwargs)

      return wrapper

    return self.HOOK(target, interFace)

  def applyHooks(self, owner: type) -> None:
    """This method applies all hooks to the instance. """
    for target, hook in self.__hooked_methods__.items():
      decorated = hook['decorated']
      interFace = maybe(hook['interFace'], Nunc)
      if hasattr(owner, target):
        original = getattr(owner, target)
        if callable(original):
          if hasattr(original, '__func__'):
            original = original.__func__
        else:
          e = typeMsg('original', original, Callable)
          raise TypeError(e)
        setattr(owner, target, interFace(decorated, original))
      else:
        setattr(owner, target, decorated)
