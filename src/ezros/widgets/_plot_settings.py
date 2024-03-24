"""The plot defaults dialog for the live plot."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QDialog, QGridLayout
from attribox import AttriBox
from ezside.core import parseParent
from ezside.widgets import BaseWidget

from ezros.widgets import SpinBox


class PlotField:
  """Descriptor protocol opening the relevant dialog on the __get__."""

  __field_name__ = None
  __field_owner__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Set the name of the field and owner."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def __get__(self, instance: PlotSettings, owner: type) -> Any:
    """Open the dialog."""
    if instance is None:
      return self
    dialog = PlotSettings()
    dialog.exec()
    return dialog.getValues()


class PlotSettings(QDialog):
  """The plot defaults dialog for the live plot."""

  baseLayout = AttriBox[QGridLayout]()
  baseWidget = AttriBox[BaseWidget]()
  vMin = AttriBox[SpinBox]('Min.', -1, 0, 1)
  vMax = AttriBox[SpinBox]('Max.', 9, 10, 11)
  hMin = AttriBox[SpinBox]('Min.', -1, 0, 1)
  hMax = AttriBox[SpinBox]('Max.', 9, 10, 11)

  def __init__(self, *args) -> None:
    """Initialize the dialog."""
    parent = parseParent()
    QDialog.__init__(self, parent)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.vMin, 1, 0)
    self.baseLayout.addWidget(self.vMax, 2, 0)
    self.baseLayout.addWidget(self.hMin, 0, 1)
    self.baseLayout.addWidget(self.hMax, 0, 2)
    self.setLayout(self.baseLayout)

  def getValues(self) -> dict:
    """Return the values."""
    return {
      'vMin': self.vMin.inner.value(),
      'vMax': self.vMax.inner.value(),
      'hMin': self.hMin.inner.value(),
      'hMax': self.hMax.inner.value()
    }
