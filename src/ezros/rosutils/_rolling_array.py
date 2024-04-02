"""RollingArray to keep visual representation classes subservient to the
actual data structure classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from msgs.msg import Float32Stamped
import numpy as np
from rospy import Time
from vistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class RollingArray():
  """RollingArray to keep visual representation classes subservient to the
  actual data structure classes. """

  __fallback_num_points__ = 16
  __zero_index__ = None
  __num_points__ = None
  __inner_array__ = None
  __zero_time__ = None

  def __init__(self, *args) -> None:
    self.__zero_time__ = Time.now().to_sec()
    intArgs = [arg for arg in args if isinstance(arg, int)]
    self._numPoints = [*intArgs, self.__fallback_num_points__][0]
    self._createInnerArray()

  def _createInnerArray(self) -> None:
    """Creates the inner array"""
    if self.__zero_index__ is not None:
      raise ValueError('The inner array has already been created')
    self.__zero_index__ = 0
    self.__inner_array__ = np.zeros((self._numPoints,), dtype=np.complex64)
    # self.__inner_array__ = np.full_like(zeros, np.nan, dtype=np.complex64)

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

  # def rightNow(self) -> tuple[np.ndarray, np.ndarray]:
  #   """Return the current array"""
  #   withNans = self._getInnerArray()
  #   yeetNans = withNans[~np.isnan(withNans)]
  #   rolled = np.roll(yeetNans, -self.__zero_index__, )
  #   times = rolled.real.astype(np.float32)
  #   values = rolled.imag.astype(np.float32)
  #   return times, values

  def explicitAppend(self, complexNumber: complex) -> None:
    """Append a value to the array"""
    self.__inner_array__[self.__zero_index__] = complexNumber
    self.__zero_index__ += 1
    if self.__zero_index__ >= self._numPoints:
      self.__zero_index__ = 0

  #
  # def append(self, data: Float32Stamped) -> None:
  #   """Append a value to the array"""
  #   value = data.data
  #   timeStamp = data.header.stamp.to_secs() - self.__zero_time__
  #   ic(timeStamp)
  #   self.__inner_array__[self.__zero_index__] = timeStamp + 1j * value
  #   self.__zero_index__ += 1
  #   if self.__zero_index__ >= self._numPoints:
  #     self.__zero_index__ = 0

  # def __str__(self) -> str:
  #   """Returns the average time and value of the snap"""
  #   t, x = self.rightNow()
  #   tMin, tMax = t.min(), t.max()
  #   xMin, xMax = x.min(), x.max()
  #   msg = """%s: %s, %s: %s, %s: %s, %s: %s""" % (
  #     'tMin', tMin, 'tMax', tMax, 'xMin', xMin, 'xMax', xMax)
  #   return msg
  #
  # def appendVals(self, time: float, value: float) -> None:
  #   """Append a value to the array"""
  #   value = 0. if value != value else value  # Trust me bro
  #   prev = self.__inner_array__[self.__zero_index__]
  #   self.__zero_index__ += 1
  #   if self.__zero_index__ >= self._numPoints:
  #     self.__zero_index__ = 0
  #   self.__inner_array__[self.__zero_index__] = time + 1j * value -
  #   prev.real

  def complexNow(self) -> np.ndarray:
    """Return the current array"""
    return np.roll(self.__inner_array__, -self.__zero_index__, )
