"""TextField subclasses TypedField providing a str valued descriptor
class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from morevistutils.fields import TypedField


class TextField(TypedField):
  """TextField subclasses TypedField providing a str valued descriptor
  class."""

  __default_text_fallback__ = 'Hello World!'

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the TextField instance."""
    defVal = None
    for arg in args:
      if isinstance(arg, str) and defVal is None:
        defVal = arg
        break
    else:
      defVal = self.__default_text_fallback__
    TypedField.__init__(self, str, defVal, *args, **kwargs)

  def __get__(self, instance: Any, owner: type, **kwargs) -> str:
    """Getter function for the field"""
    return TypedField.__get__(self, instance, owner)
