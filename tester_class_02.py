"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from tester_class_01 import Tester, Tester2, Taboo

ic.configureOutput(includeContext=True)


class LOL(type):
  """LOL provides a metaclass that can be used as a base class for
  classes"""
  +Taboo
  print('%s LOL body' % Taboo)

  def __call__(cls, cls_: type) -> type:
    +Taboo
    print('%s LOL metaclass: (%s)' % (Taboo, cls_.__qualname__))
    return cls_


class Decorator(metaclass=LOL):
  """Decorator provides a class that can be used as a base class for classes
  that own fields."""


class Field:
  """Field provides a class that can be used as a base class for classes
  that own fields."""

  def __set_name__(self, owner, name: object):
    +Taboo
    ownerName = owner.__qualname__
    fieldName = 'Field'
    print('%s %s.__set_name__(%s, %s)' % (Taboo, 'Field', ownerName, name))


class Meta(type):
  """Meta provides a metaclass that can be used as a base class for
  classes"""

  +Taboo
  print('%s Meta body' % Taboo)

  @classmethod
  def __prepare__(mcls, name, bases, **kwargs) -> dict:
    +Taboo
    print('%s %s.__prepare__(%s, )' % (Taboo, 'Meta', name,))
    return {}

  def __new__(mcls, name, bases, ns, **kwargs) -> type:
    +Taboo
    print('%s %s.__new__(%s, )' % (Taboo, 'Meta', name,))
    return type.__new__(mcls, name, bases, ns)

  def __init__(cls, name, bases, ns, **kwargs) -> None:
    +Taboo
    print('%s %s.__init__(%s, )' % (Taboo, 'Meta', name,))

  def __call__(cls, *args, **kwargs) -> type:
    +Taboo
    name = cls.__qualname__
    print('%s %s.__init__(%s, )' % (Taboo, 'Meta', name,))
    return type.__call__(cls, *args, **kwargs)


@Decorator
class Owner(metaclass=Meta):
  """Owner provides a class that can be used as a base class for classes
  that own fields."""
  pass

  a = Field()
  b = Field()

  def __init__(self, *args, **kwargs) -> None:
    +Taboo
    msg = [*[arg for arg in args if isinstance(arg, str)], ''][0]
    print('%s %s.__init__()' % (Taboo, 'Owner'))

  def __str__(self) -> str:
    return self.__class__.__qualname__
