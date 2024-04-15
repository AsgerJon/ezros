"""Button provides a simple pushbutton"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.core import Tight, Expand
from ezside.widgets import BaseWidget, Vertical, Horizontal
from icecream import ic
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg


class Button(BaseWidget):
  """Button provides a simple pushbutton"""
  __fallback_label__ = 'Click Me!'
  __label_text__ = None

  __inner_button__ = None
  baseLayout = AttriBox[Horizontal]()
  clicked = Signal()

  def _setLabel(self, label: str) -> None:
    """Setter-function for label-text"""
    if isinstance(label, str):
      self.__label_text__ = label
      return self._getButton().setText(label)
    e = typeMsg('label', label, str)
    raise TypeError(e)

  def _getLabel(self) -> str:
    """Getter-function for label-text"""
    label = maybe(self.__label_text__, self.__fallback_label__)
    if isinstance(label, str):
      return label
    e = typeMsg('label', label, str)
    raise TypeError(e)

  def _getButton(self, **kwargs) -> QPushButton:
    """Returns the button."""
    if self.__inner_button__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      kwargs['_recursion'] = True
      self.__inner_button__ = QPushButton(self)
      self.__inner_button__.setText(self._getLabel())
      return self._getButton(**kwargs)
    return self.__inner_button__

  def initUi(self) -> None:
    """The initUi method initializes the user interface of the window."""
    self.baseLayout.addWidget(self._getButton())
    self.setLayout(self.baseLayout)
    self.setSizePolicy(Tight, Expand)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the Button instance."""
    BaseWidget.__init__(self, *args, **kwargs)
    ic(*args)
    for arg in args:
      if isinstance(arg, str):
        self._setLabel(arg)
        break
    else:
      self._setLabel(self.__fallback_label__)

  def connectActions(self) -> None:
    """Connect the actions."""
    self._getButton().clicked.connect(self.clicked)
