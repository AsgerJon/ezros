"""ArrayField wraps the numpy array in a closure and provides a descriptor
implementation."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Self, Any

import numpy as np
from icecream import ic
from vistutils.fields import unParseArgs, Wait
from vistutils.waitaminute import typeMsg


class Array:
  """The array provides the inner array of the ArrayField."""

  __inner_data__ = None
  __first_index__ = 0
  __zero_time__ = time.time()
  __fallback_num_points__ = 128
  __num_points__ = None

  def _initData(self, ) -> None:
    """Initializes the array with the given data."""
    n = self.__num_points__
    self.__inner_data__ = np.linspace(0, 1, n - 1, dtype=complex)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the array."""
    self.__num_points__ = self.__fallback_num_points__
    self.__zero_time__ = time.time()
    self._initData()

  def setNumPoints(self, n: int) -> None:
    """Sets the number of points in the array."""
    raise NotImplementedError

  def snap(self, ) -> np.ndarray:
    """Returns a copy of the array at the moment of the call."""
    copyData = np.copy(self.__inner_data__)
    np.roll(copyData, -self.__first_index__)
    return copyData - self.__zero_time__

  def append(self, value: float) -> None:
    """Appends the value to the array."""
    if value != value:  # super cursed, but filters nans conveniently
      value = 0
    self.__zero_time__ = self.__inner_data__[self.__first_index__].real
    self.__inner_data__[self.__first_index__] = time.time() + value * 1j
    n = self.__inner_data__.shape[0]
    self.__first_index__ = (self.__first_index__ + 1) % n
