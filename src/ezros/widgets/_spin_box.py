"""SpinBox widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QDoubleSpinBox, QHBoxLayout
from attribox import AttriBox
from ezside.widgets import BaseWidget, TextLabel
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg


class SpinBox(BaseWidget):
  """SpinBox widget. """

  inner = AttriBox[QDoubleSpinBox]()
  baseLayout = AttriBox[QHBoxLayout]()
  label = AttriBox[TextLabel]()

  newValue = Signal()

  def __init__(self, *args, ) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, )
    minValue, value, maxValue = None, None, None
    title = None
    values = []
    for arg in args:
      if isinstance(arg, str) and title is None:
        title = arg
      elif isinstance(arg, float):
        values.append(arg)
      if len(values) > 2 and title is not None:
        break
    else:
      self.label.setText(maybe(title, ''))
      if not values:
        e = """Must receive at least one value!"""
        raise ValueError(e)
      if len(values) == 1:
        value = values[0]
        if value < 0:
          minValue = - max(abs(value), 1)
          maxValue = 0
        else:
          minValue = 0
          maxValue = max(value, 1)
      elif len(values) == 2:
        value = values[0]
        if value < values[1]:
          dV = values[1] - value
          minValue = value - dV
          maxValue = values[1]
        else:
          dV = value - values[1]
          minValue = values[1]
          maxValue = value + dV
      elif len(values) == 3:
        minValue, value, maxValue = values
      else:
        e = """Too many values!"""
        raise ValueError(e)
    self.__min_value__ = minValue
    self.__spin_value__ = value
    self.__max_value__ = maxValue
    self.__label_title__ = title

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.inner.setRange(self.__min_value__, self.__max_value__)
    self.inner.setValue(self.__spin_value__)
    self.label.setText(self.__label_title__)
    self.baseLayout.addWidget(self.label)
    self.baseLayout.addWidget(self.inner)
    self.setLayout(self.baseLayout)
    self.inner.valueChanged.connect(self.newValue.emit)
