"""The OverloadNamespace provides the namespace object used by the
OverloadMeta metaclass. It subclasses AbstractNamespace making use of the
two sided namespace pattern, where one side is used during execution of
the class body, which is then compiled into a regular 'dict' instance,
which is then used by the '__new__ method to create the new class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.metas import AbstractNamespace


class OverloadNamespace(AbstractNamespace):
  """The OverloadNamespace provides the namespace object used by the
  OverloadMeta metaclass. It subclasses AbstractNamespace making use of the
  two sided namespace pattern, where one side is used during execution of
  the class body, which is then compiled into a regular 'dict' instance,
  which is then used by the '__new__ method to create the new class. """

  def getCallables(self) -> dict:
    """This method returns a dictionary of callables that are to be
    overloaded. """
    callableNamespace = {}
    for item in self.getLog():
      acc, key, val = item['acc'], item['key'], item['val']
      if acc == 'set' and callable(val):
        existing = callableNamespace.get(key, [])
        callableNamespace |= {key: [*existing, val]}
    return callableNamespace

  def compile(self) -> dict:
    """This method creates the actual namespace object sent to the
    '__new__' in the super call. It leverages the '__access_log__' of the
    parent class to collect the overloaded functions. """
