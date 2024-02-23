"""AbstractBaseField provides a baseclass for descriptor classes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any


class AbstractBaseField:
  """AbstractBaseField provides a baseclass for descriptor classes."""

  __owned_instances__ = {}

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

  def HOOK(self, target: str, interFace: Callable = None) -> None:
    """Methods decorated with this decorator, will apply 'interFace' to
    the callable named 'target' in the owner class. If 'decorated' is a
    method in the descriptor class, and 'original' is the method at the
    target name in the owner class, then:

    setattr(owner, target, interFace(target, original))"""
