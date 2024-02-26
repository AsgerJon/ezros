"""Nunc is the NULL version of the callable. It takes any number of
positional and keyword arguments and returns None. It is always Falsy and
always a singleton instance of the Nunc type."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from icecream import ic

from morevistutils.metas import Singleton


@Singleton
class NuncType:
  """Nunc is the NULL version of the callable. It takes any number of
  positional and keyword arguments and returns them. It is always Falsy and
  always a singleton instance of the Nunc type."""

  def __init_subclass__(cls, **kwargs) -> Never:
    """Illegal to subclass NuncType!"""
    if '_root' not in kwargs:
      raise TypeError("NuncType is a singleton and cannot be subclassed!")

  def __call__(self, *args, **kwargs) -> Any:
    if args:
      return args[0]

  def __bool__(self, ) -> bool:
    return False

  def __eq__(self, *_) -> Never:
    """Nunc is always different from anything else!"""
    raise TypeError("Nunc is always different from anything else!")


Nunc = NuncType()
