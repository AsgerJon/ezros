"""The DataEcho class provides a way to echo data from one object to
another."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from random import random
from typing import Any

import numpy as np
from PySide6.QtCore import QPointF, QPoint, QSize, QSizeF, QRectF, QRect
from icecream import ic
from numpy import ndarray, nan, complex64, full
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg


class DataEcho:
  """The DataEcho class provides a way to echo data from one object to
  another."""
  __inner_array__ = None
  __unit_values__ = None
  __zero_position__ = 0
  __fallback_epoch__ = 128

  def __init__(self, *args, **kwargs) -> None:
    self.__unit_values__ = np.ones_like(self, dtype=np.float64)
    self.__zero_position__ = 0
    n = None
    for arg in args:
      if isinstance(arg, int):
        self.__epoch_length__ = arg
        break
    else:
      self.__epoch_length__ = self.__fallback_epoch__
    self.__inner_array__ = np.zeros(self.__epoch_length__,
                                    dtype=np.complex128)
    for i in range(self.__epoch_length__):
      val = random()
      self.append(val)

  def __len__(self, *args, **kwargs) -> int:
    """Return the length of the DataEcho object."""
    return self.__inner_array__.shape[0]

  def append(self, value: float = None) -> None:
    """Append a value to the DataEcho object."""
    if not value == value:
      value = 0j
    if isinstance(value, int):
      value = float(value) + 0j
    if isinstance(value, complex):
      value = value.imag
    self.__inner_array__[self.__zero_position__] = time.time() + value * 1j
    self.__zero_position__ += 1
    self.__zero_position__ %= self.__epoch_length__

  def snapshot(self) -> ndarray:
    """Return a snapshot of the DataEcho object."""
    out = self.__inner_array__.copy()
    return out - full(len(self), complex(min(out.real) + 0j))

  def tolist(self, ) -> list[complex]:
    """Return the DataEcho object as a list."""
    return self.__inner_array__.tolist()

  def __str__(self) -> str:
    """Return a string representation of the DataEcho object."""
    w = np.ceil(len(self))
    vals = self.tolist()
    lines = ['%.3f + %.3f*j' % (val.real, val.imag) for val in vals]
    return '\n'.join(lines)

  def __matmul__(self, other: Any) -> list[QPointF]:
    """Creates a new array scaled to other"""
    w, h = 1, 1
    T0, X0 = 0, 0
    if isinstance(other, (QPoint, QPointF)):
      w, h = other.x(), other.y()
    elif isinstance(other, (QSize, QSizeF)):
      w, h = other.width(), other.height()
    elif isinstance(other, (QRect, QRectF)):
      T0, X0 = other.left(), other.top()
      w, h = other.width(), other.height()

    T, X = self.__inner_array__.real, self.__inner_array__.imag
    T -= min(T)
    T /= max(T)
    X -= min(X)
    X /= max(X)

    out = []
    for t, x in zip(w * T, h * X):
      if t == t:
        t_ = t + T0
      else:
        t_ = 0
      if x == x:
        x_ = x + X0
      else:
        x_ = 0

      out.append(QPointF(t + T0, x + X0))
      ic(t + T0, x + X0)

    return out
