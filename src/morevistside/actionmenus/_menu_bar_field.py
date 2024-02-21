"""MenuBarField is a descriptor class placing a menu bar on a QMainWindow"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Type, Any, Callable

from PySide6.QtWidgets import QMainWindow
from vistutils.fields import AbstractField


class MenuBarField(AbstractField):
  """MenuBarField is a descriptor class placing a menu bar on a
  QMainWindow"""

  def _setupFactory(self, owner: Type[QMainWindow]) -> Callable:
    """Sets up the factory"""

    def setupField(this: Any, value: Any) -> None:
      """Sets up the field"""

    return setupField

  def __prepare_owner__(self, owner: Type[QMainWindow]) -> None:
    """Implementation of the abstract method"""
