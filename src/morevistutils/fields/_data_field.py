"""DataField contains roller data for use in plotting of data."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

import numpy as np
from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField


class DataField(AbstractBaseField):
  """DataField contains roller data for use in plotting of data."""

  __array_length__ = None
  __fallback_length__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the DataField"""
    AbstractBaseField.__init__(self, **kwargs)
    for arg in args:
      if isinstance(arg, int) and self.__array_length__ is None:
        self.__array_length__ = arg
        break
    else:
      self.__array_length__ = self.__fallback_length__

  def createArray(self, ) -> np.ndarray:
    """Creates an array of the correct length"""
    dataArray = np.zeros(self.__array_length__, dtype=np.complex128)
    for i in range(self.__array_length__):
      dataArray[i] = time.time() + 0j
    return dataArray

  def __get__(self, instance: Any, owner: type) -> Any:
    """Get the value of the field."""
    if instance is None:
      return self
    pvtName = self.getPrivateName()
    if getattr(instance, pvtName, None) is not None:
      val = getattr(instance, pvtName)
      if isinstance(val, np.ndarray):
        return val
      e = typeMsg('val', val, np.ndarray)
      raise TypeError(e)
    raise NotImplementedError
