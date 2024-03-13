"""The circular buffer provides a performant storage of data. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


import numpy as np

class CircularBuffer:
  """The CircularBuffer class provides a performant storage of data."""

  def __init__(self, capacity) -> None:
    self.data = np.zeros(capacity)
    self.capacity = capacity
    self.start = 0
    self.size = 0

  def initData(self, value: float) -> None:
    """Initializes the buffer with the given data."""
    self.data.fill(value)
    self.size = self.capacity

  def append(self, value: float):
    end = (self.start + self.size) % self.capacity
    self.data[end] = value
    self.size = min(self.size + 1, self.capacity)
    self.start = (self.start + 1) % self.capacity
