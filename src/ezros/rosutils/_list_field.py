"""Descriptor class for list"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Never

from PySide6.QtCharts import QChartView
from PySide6.QtCore import QPointF, Slot
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FloatField
from vistutils.waitaminute import typeMsg


class ListLike:
  """Strongly typed list"""

  __inner_contents__ = None
  __iter_contents__ = None
  __field_type__ = None

  def __init__(self, type_: type) -> None:
    if type_ is None:
      e = """Instances of '%s' must be initialized with a type!"""
      raise TypeError(e % self.__class__.__name__)
    if not isinstance(type_, type):
      e = typeMsg('type_', type_, type)
      raise TypeError(e)
    self.__field_type__ = type_

  def __len__(self, ) -> int:
    """Returns the length of the list."""
    if self.__inner_contents__ is None:
      return 0
    return len(self.__inner_contents__)

  def __iter__(self, ) -> iter:
    """Returns an iterator for the list."""
    self.__iter_contents__ = [*(self.__inner_contents__ or []), ]
    return self

  def __next__(self, ) -> Any:
    """Returns the next element in the list."""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration

  def __contains__(self, item: Any) -> bool:
    """Returns True if the item is in the list."""
    if isinstance(item, self.__field_type__):
      return item in (self.__inner_contents__ or [])
    return False

  def _rollIndex(self, index: int) -> int:
    """Returns the rolled index."""
    n = len(self)
    while index < 0:
      index += n
    return index % n

  def __getitem__(self, *args) -> Any:
    """Returns the item at the given index."""
    if self.__inner_contents__ is None:
      e = """Index out of range!"""
      raise IndexError(e)
    intArgs = [arg for arg in args if isinstance(arg, int)]
    if len(intArgs) == 1:
      if isinstance(intArgs[0], int):
        return self.__inner_contents__[self._rollIndex(intArgs[0])]
    if len(intArgs) == 2:
      if isinstance(intArgs[0], int) and isinstance(intArgs[1], int):
        a, b = [self._rollIndex(arg) for arg in intArgs]
        if a > b:
          return self.__inner_contents__[a:] + self.__inner_contents__[:b]
        return self.__inner_contents__[a:b]
    for arg in args:
      callable(arg)
      out = []
      for item in self.__inner_contents__:
        if arg(item):
          out.append(item)
      return out

  def __setitem__(self, *_) -> Never:
    """Illegal setter"""
    e = """Instances of ListLike are read-only!"""
    raise TypeError(e)

  def append(self, item: Any) -> None:
    """Appends an item to the list."""
    if self.__inner_contents__ is None:
      self.__inner_contents__ = []
    if isinstance(item, self.__field_type__):
      self.__inner_contents__.append(item)
    else:
      e = typeMsg('item', item, self.__field_type__)
      raise TypeError(e)

  def __add__(self, other: Any) -> ListLike:
    """Returns the concatenation of the list with another list."""
    try:
      self.append(other)
    except TypeError:
      return NotImplemented


class ListField:
  """Descriptor class for list"""

  __field_name__ = None
  __field_owner__ = None
  __field_type__ = None

  def __init__(self, type_: type) -> None:
    if type_ is None:
      e = """Instances of '%s' must be initialized with a type!"""
      raise TypeError(e % self.__class__.__name__)
    if not isinstance(type_, type):
      e = typeMsg('type_', type_, type)
      raise TypeError(e)
    self.__field_type__ = type_

  def __set_name__(self, owner: type, name: str) -> None:
    """Invoked automatically when the owner class is created."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateName(self) -> str:
    """Returns the private name of the field."""
    return '_%s' % self.__field_name__

  def __get__(self, instance: object, owner: type, **kwargs) -> Any:
    """Returns the value of the descriptor."""
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if getattr(instance, pvtName, None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      setattr(instance, pvtName, ListLike(self.__field_type__))
      return self.__get__(instance, owner, _recursion=True)
    return getattr(instance, pvtName)

  def __set__(self, *_) -> Never:
    """Illegal setter function"""
    e = """Instances of ListField are read-only!"""
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal deleter function"""
    e = """Instances of ListField are read-only!"""
    raise TypeError(e)

  def __str__(self, ) -> str:
    """Returns a string representation of the descriptor."""
    return """%s.%s: list[%s]""" % (self.__field_owner__.__name__,
                                    self.__field_name__,
                                    self.__field_type__.__name__)
