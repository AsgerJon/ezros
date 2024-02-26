"""The builtin 'object' type in Python provides a common  baseclass for
every single object. Unfortunately, it does exhibit 'Karen'-type behaviour
by raising exceptions, when methods are not explicitly reimplemented to
absorb arguments, these arguments reach the object class where they raise
'Karen'-type exceptions.

The reasoning behind this may be sound, (not really, but whatever),
in production code to prevent something or something, but during
development, it is convenient to be allowed to pass *args and **kwargs up
through a chain of classes without having to prevent these from reaching
'Karen'."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class ObjectType(object):
  """Go away, Karen!"""

  def __init__(self, *args, **kwargs) -> None:
    """If you never implement __init__ and you allow the instances to use
    object.__init__, you will not be able to pass any arguments up through
    the chain of classes. This is by intention of course, for some reason,
    but during development it is stupid. Instead, have at least a method
    absorbing the arguments. This way, later on you can include arguments
    in the inheritance chain without having to change a bunch of code."""

  def __init_subclass__(cls, **kwargs) -> None:
    """Note from the author of this code: I have personally lost 100 hours
    because of:
      TypeError: object.__init_subclass__() takes no keyword arguments
    This error causes far more confusion than it prevents."""
