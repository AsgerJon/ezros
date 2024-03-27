"""QAttriBox class for handling QAttriBox objects in ROS."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from vistutils.waitaminute import typeMsg


class QAttriBox(AttriBox):
  """QAttriBox class for handling QAttriBox objects in ROS."""

  __reference_list__ = None

  def getReferenceList(self, **kwargs) -> list:
    """Returns the reference list."""
    if getattr(self, '__reference_list__', None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.__reference_list__ = []
      return self.getReferenceList(_recursion=True)
    if isinstance(self.__reference_list__, list):
      return self.__reference_list__
    e = typeMsg('self.__reference_list__', self.__reference_list__, list)
    raise TypeError(e)

  def referInstance(self, innerInstance: object) -> object:
    """Refer an instance."""
    self.getReferenceList().append(innerInstance)
    return innerInstance

  def _createInnerObject(self, instance: object) -> object:
    """Creates an instance of the inner class. """
    return self.referInstance(AttriBox._createInnerObject(self, instance))
