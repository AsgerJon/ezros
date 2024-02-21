"""BaseWidget provides the lowest level of abstraction for a QWidget. It is
controls only size limits."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget, QSizePolicy
from vistutils.fields import TypedField

MinExp = QSizePolicy.Policy.MinimumExpanding
Max = QSizePolicy.Policy.Maximum


class BaseWidget(QWidget):
  """BaseWidget provides the lowest level of abstraction for a QWidget. It is
  controls only size limits."""

  minWidth = TypedField(int, 32)
  maxWidth = TypedField(int, 1024)
  minHeight = TypedField(int, 32)
  maxHeight = TypedField(int, 768)

  def __init__(self, *args, **kwargs) -> None:
    QWidget.__init__(self, *args, **kwargs)
    self.setMinimumSize(self.minWidth, self.minHeight)
    self.setMaximumSize(self.maxWidth, self.maxHeight)
    self.setSizePolicy(MinExp, Max)
