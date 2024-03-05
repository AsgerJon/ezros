"""The dataMap module provides a fast transfer of numpy arrays from the
data space to pixel space."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from numba import jit
import numpy as np

Array = np.ndarray


@jit(nopython=True)
def dataMap(x: np.ndarray, xNewMin: float, xNewMax: float, ) -> Array:
  """
  Scale x and y numpy arrays from their original range to a new range
  using Numba JIT for performance.

  Parameters:
  - x, y: Original arrays of x and y coordinates.
  - xNewMin, xNewMax: The new minimum and maximum values for the x
  coordinates.
  - yNewMin, yNewMax: The new minimum and maximum values for the y
  coordinates.

  Returns:
  - A tuple of scaled (x, y) arrays.
  """
  xMin, xMax = np.min(x), np.max(x)
  xScaled = xNewMin + (x - xMin) * (xNewMax - xNewMin) / (xMax - xMin)
  return xScaled
