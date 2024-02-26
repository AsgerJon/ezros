"""AbstractPaint provides the abstract baseclass for the encapsulated
painting operations"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QPainter, QPaintEvent


class AbstractPaint:
  """AbstractPaint provides the abstract baseclass for the encapsulated
  painting operations"""

  @abstractmethod
  def paintOp(self, event: QPaintEvent, painter: QPainter) -> None:
    """Applies the paint operation"""
