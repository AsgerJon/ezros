"""Flag provides a descriptor class for boolean states."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from vistutils.waitaminute import typeMsg

from morevistutils.fields import FlexField


class Flag(FlexField):
  """Flag provides a descriptor class for boolean states."""

  def getGetter(self, ) -> Callable:
    """This reimplementation skips the default getter. Instead the getter
    should reflect a state defined on the owner class. """
    if self.__explicit_getter__ is None:
      e = 'No getter function set.'
      raise AttributeError(e)
    if callable(self.__explicit_getter__):
      return self.__explicit_getter__
    e = typeMsg('getter', self.__explicit_getter__, Callable)
    raise TypeError(e)

  def __get__(self, instance: Any, owner: Any) -> Any:
    getter = self.getGetter()
    return True if getter(instance, owner) else False
