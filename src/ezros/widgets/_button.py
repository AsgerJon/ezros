"""Button provides a simple pushbutton"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget, Vertical


class Button(BaseWidget):
  """Button provides a simple pushbutton"""
  __inner_button__ = None
  baseLayout = AttriBox[Vertical]()
  clicked = Signal()

  def _getButton(self, **kwargs) -> QPushButton:
    """Returns the button."""
    if self.__inner_button__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      kwargs['_recursion'] = True
      self.__inner_button__ = QPushButton(self)
      self.__inner_button__.clicked.connect(self.clicked)
      return self._getButton(**kwargs)
    return self.__inner_button__

  def initUi(self) -> None:
    """The initUi method initializes the user interface of the window."""
    self.baseLayout.addWidget(self._getButton())
    self.setLayout(self.baseLayout)
