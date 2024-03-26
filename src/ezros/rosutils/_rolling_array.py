"""RollingArray to keep visual representation classes subservient to the
actual data structure classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from icecream import ic
import numpy as np
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class RollingArray:
  """RollingArray to keep visual representation classes subservient to the
  actual data structure classes. """

  __fallback_num_points__ = 16
  __zero_index__ = None
  __num_points__ = None
  __inner_array__ = None

  def __init__(self, numPoints: int = None) -> None:
    self.__zero_index__ = 0
    self.__num_points__ = maybe(numPoints, self.__fallback_num_points__)
    self._list = []

  def _createInnerArray(self) -> None:
    """Creates the inner array"""
    now = time.time()
    then = now + 100
    num = self.__num_points__
    reals = np.linspace(now, then, num, dtype=np.complex64)
    fakes = np.zeros((num,), dtype=np.complex64)
    self.__inner_array__ = (reals + fakes).astype(np.complex64)

  def _getInnerArray(self, **kwargs) -> np.ndarray:
    """Getter-function for the inner array"""
    if self.__inner_array__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createInnerArray()
      return self._getInnerArray(_recursion=True)
    if isinstance(self.__inner_array__, np.ndarray):
      return self.__inner_array__
    e = typeMsg('self.__inner_array__', self.__inner_array__, np.ndarray)
    raise TypeError(e)

  def getZero(self) -> int:
    """Return the zero index"""
    return self.__zero_index__

  def incZero(self) -> int:
    """Increment the zero index"""
    self.__zero_index__ = (self.__zero_index__ + 1) % self.__num_points__
    return self.__zero_index__ - 1

  def rightNow(self) -> tuple[np.ndarray, np.ndarray]:
    """Return the current array"""
    base = np.roll(self._getInnerArray(), -self.__zero_index__, )
    times = base.real
    times -= max(times)
    values = base.imag

    return times.astype(np.float32), values.astype(np.float32)

  def append(self, value: float) -> None:
    """Append a value to the array"""
    self.__inner_array__[self.__zero_index__] = self.moment() + value * 1j
    self.__zero_index__ += 1
    if self.__zero_index__ >= self.__num_points__:
      self.__zero_index__ = 0

  @staticmethod
  def moment() -> float:
    """Return the current time"""
    return time.time()

  def __str__(self) -> str:
    """Returns the average time and value of the snap"""
    t, x = self.rightNow()
    tMin, tMax = t.min(), t.max()
    xMin, xMax = x.min(), x.max()
    msg = """%s: %s, %s: %s, %s: %s, %s: %s""" % (
      'tMin', tMin, 'tMax', tMax, 'xMin', xMin, 'xMax', xMax)
    return msg