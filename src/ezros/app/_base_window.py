"""BaseWindow class provides menus and actions for the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QMainWindow
from attribox import AttriBox, this
from ezside import MenuBar, StatusBar
from ezside.windows import BaseWindow as Base
from icecream import ic

ic.configureOutput(includeContext=True)


class BaseWindow(Base):
  """BaseWindow class provides menus and actions for the application."""

  def initUi(self) -> None:
    """Initialize the user interface."""
    Base.initUi(self)

  def initActions(self) -> None:
    """Initialize the actions."""
    Base.initActions(self)
