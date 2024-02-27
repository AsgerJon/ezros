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
  __zero_time__ = None

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
    self._first = True

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
    if self.__zero_time__ is None:
      self.__zero_time__ = time.time()
      stampedValue = self.__zero_time__ + value * 1j
    stampedValue = time.time() - self.__zero_time__ + value * 1j
    self.__inner_array__[self.__zero_position__] = stampedValue
    self.__zero_position__ += 1
    self.__zero_position__ %= self.__epoch_length__

  def snapshot(self) -> tuple[ndarray, ndarray]:
    """Return a snapshot of the DataEcho object."""
    out = self.__inner_array__.copy()
    T, X = out.real, out.imag
    minT, maxT, minX, maxX = min(T), max(T), min(X), max(X)
    return T, X

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
    if isinstance(other, (QRect, QRectF)):
      T0, X0 = other.left(), other.top()
      w, h = other.width(), other.height()
    elif isinstance(other, (QSize, QSizeF)):
      w, h = other.width(), other.height()
    elif isinstance(other, (QPoint, QPointF)):
      w, h = other.x(), other.y()

    T, X = self.snapshot()

    T -= min(T)
    T /= max(T)
    X -= min(X)
    X /= max(X)

    T, X = T * w, X * h

    out = []
    for t, x in zip(T, X):
      out.append(QPointF(t + T0, x + X0))

    return out

  def __getattr__(self, key: str) -> Any:
    """Please note that this method was implemented by an experienced
    professional. Do not try this at home!"""
    if key == '__inner_array__':
      raise AttributeError("""Attribute '%s' not found!""" % key)
    innerArray = object.__getattribute__(self, '__inner_array__')
    try:
      return object.__getattribute__(innerArray, key)
    except AttributeError as attributeError:
      e = """Attribute '%s' not found in DataEcho object!"""
      raise AttributeError(e % key) from attributeError

  def horizontalTicks(self, n: int, span: float, offset: float) -> list:
    """This method returns a list of tuples where the first element is the
    value in the data and the second element is the pixel position. """
    T, X = self.snapshot()
    tSpan = max(T) - min(T)
    uT = T - min(T)
    uT /= max(uT)
    pT = offset + uT * span
    timeStep = tSpan / n
    pixelStep = span / n
    out = []
    for i in range(n + 1):
      value = -tSpan + i * timeStep
      pixel = i * pixelStep + offset
      out.append((value, pixel))
    return out

  def verticalTicks(self, n: int, span: float, offset: float) -> list:
    """This method returns a list of tuples where the first element is the
    value in the data and the second element is the pixel position. """
    T, X = self.snapshot()
    xSpan = max(X) - min(X)
    uX = X - min(X)
    uX /= max(uX)
    pX = offset + uX * span
    valueStep = xSpan / n
    pixelStep = span / n
    out = []
    for i in range(n + 1):
      value = min(X) + i * valueStep
      pixel = span + offset - i * pixelStep
      out.append((value, pixel))
    return out
