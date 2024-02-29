"""BaseLineEdit wraps the QLineEdit class allowing it to interact with the
EZRos gui system."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import QWidget, QLineEdit

from ezros.gui.shortnames import parseParent


class BaseLineEdit(QLineEdit):
  """The BaseLineEdit class wraps the QLineEdit class allowing it to interact
  with the EZRos gui system. This class should be used as a base for all line
  edit widgets in the EZRos system."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the BaseLineEdit class."""
    parent = parseParent(*args, )
    QLineEdit.__init__(self, parent)
