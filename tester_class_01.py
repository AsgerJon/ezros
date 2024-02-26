"""Tester class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils.metas import Singleton


@Singleton
class Tester:
  """Tester class"""

  def __init__(self, *args, **kwargs) -> None:
    self.__msg__ = [*[arg for arg in args if isinstance(arg, str)], 'LOL'][0]

  def __str__(self) -> str:
    return self.__msg__


class Tester2(Singleton):
  """Tester class"""

  def __init__(self, *args, **kwargs) -> None:
    self.__msg__ = [*[arg for arg in args if isinstance(arg, str)], 'LOL'][0]

  def __str__(self) -> str:
    return self.__msg__


class Bla(type):
  """Bla provides a metaclass that can be used as a base class for
  classes"""

  def __str__(cls, ) -> str:
    return '%s: %02d' % (cls.__qualname__, cls.__secret_counter__)

  def __new__(cls, *args, **kwargs) -> type:
    cls = type.__new__(cls, *args, **kwargs)
    cls.__secret_counter__ = 0
    return cls

  def __pos__(cls) -> int:
    cls.__secret_counter__ += 1
    return cls.__secret_counter__


class Taboo(metaclass=Bla):
  """Counter class"""

  __secret_counter__ = 0

  @classmethod
  def __ladd__(cls, *args, **kwargs) -> None:
    cls.__secret_counter__ += 1
