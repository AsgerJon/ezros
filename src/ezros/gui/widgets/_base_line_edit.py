"""BaseLineEdit wraps the QLineEdit class allowing it to interact with the
EZRos gui system."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import QWidget, QLineEdit
from vistutils.parse import maybe

from ezros.gui.shortnames import parseParent


class BaseLineEdit(QLineEdit):
  """The BaseLineEdit class wraps the QLineEdit class allowing it to interact
  with the EZRos gui system. This class should be used as a base for all line
  edit widgets in the EZRos system."""

  __fallback_placeholder__ = 'Enter text here...'
  __default_placeholder__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the BaseLineEdit class."""
    parent = parseParent(*args, )
    QLineEdit.__init__(self, parent)
    placeHolderKwarg = kwargs.get('placeHolder', None)
    placeHolderArg = None
    for arg in args:
      if isinstance(arg, str):
        placeHolderArg = arg
        break
    self.__default_placeholder__ = maybe(
      placeHolderKwarg, placeHolderArg, self.__fallback_placeholder__)
    self.setPlaceholderText(self.__default_placeholder__)
