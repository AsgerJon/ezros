"""BaseWidget provides the lowest level of abstraction for a QWidget. It is
controls only size limits."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QEnterEvent, QGuiApplication
from PySide6.QtWidgets import QWidget, QSizePolicy

from ezros.gui.shortnames import parseParent

MinExp = QSizePolicy.Policy.MinimumExpanding
Max = QSizePolicy.Policy.Maximum


class BaseWidget(QWidget):
  """BaseWidget provides the lowest level of abstraction for a QWidget. It is
  controls only size limits."""

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
    self.connectActions()

  def connectActions(self) -> None:
    """Subclasses should implement this method if they need to handle
    actions."""
