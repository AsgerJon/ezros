"""SpinBox widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QDoubleSpinBox, QHBoxLayout, QVBoxLayout
from attribox import AttriBox
from ezside.core import Tight
from ezside.widgets import BaseWidget, TextLabel
from icecream import ic
from vistutils.parse import maybe

from ezros.widgets import TightLabel

ic.configureOutput(includeContext=True, )


class SpinBox(BaseWidget):
  """SpinBox widget. """

  __label_title__ = None
  __min_value__ = None
  __spin_value__ = None
  __max_value__ = None

  inner = AttriBox[QDoubleSpinBox]()
  baseLayout = AttriBox[QVBoxLayout]()
  label = AttriBox[TightLabel]()

  newValue = Signal(float)
  update = Signal()

  def __init__(self, *args, ) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, )
    if not args:
      e = """Received no positional arguments"""
      raise ValueError(e)
    strArgs = [*[arg for arg in args if isinstance(arg, str)], 'SpinBox']
    floatArgs = [arg for arg in args if isinstance(arg, (int, float))]
    self.__label_title__ = strArgs[0]
    min_, val, max_ = [*floatArgs, None, None, None][:3]
    if any([min_ is None, val is None, max_ is None]):
      e = """The minimum, initial value and maximum value are required!"""
      raise ValueError(e)
    self.__min_value__ = min_
    self.__spin_value__ = val
    self.__max_value__ = max_

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.inner.setMinimumHeight(32)
    self.inner.setRange(self.__min_value__, self.__max_value__)
    self.inner.setValue(self.__spin_value__)
    self.label.setText(self.__label_title__)
    self.label.setSizePolicy(Tight, Tight)
    self.baseLayout.addWidget(self.label)
    self.baseLayout.addWidget(self.inner)
    self.setLayout(self.baseLayout)
    BaseWidget.initUi(self)

  def connectActions(self) -> None:
    """Connect actions to slots."""
    self.inner.valueChanged.connect(self.newValue.emit)
    self.inner.editingFinished.connect(self.update.emit)

  def __str__(self, ) -> str:
    """Return a string representation of the widget."""
    clsName = self.__class__.__name__
    title = self.label.getText()
    return '%s: %s' % (clsName, title)

  def __repr__(self, ) -> str:
    """Return a string representation of the widget."""
    title = self.__label_title__
    min_, max_ = self.__min_value__, self.__max_value__
    value = self.__spin_value__
    clsName = self.__class__.__qualname__
    return '%s(%s, %s, %s, %s)' % (clsName, title, min_, value, max_)
