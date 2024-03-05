"""WithNode sets a node name for the decorated class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class WithNode:
  """Sets a node name for the decorated class."""

  __decorated_classes__ = []

  def __init__(self, nodeName: str, **kwargs) -> None:
    """Initializes the decorator"""
    self._nodeName = nodeName
    self._anonymousFlag = kwargs.get('anonymous', False)

  def __call__(self, cls) -> cls:
    """Decorates the class"""
    if cls in self.__decorated_classes__:
      e = """Class '%s' is already decorated with a node name!""" % cls
      raise AttributeError(e)
