"""BaseWindow provides menus and actions for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMainWindow, QMenuBar

from ezros.gui.factories import menuBarFactory
from morevistutils.fields import Field


class _BaseWindowFields(QMainWindow):
  """Convenience class containing fields for BaseWindow."""

  mainMenuBar = Field(QMenuBar, None)
  mainMenuBar.CREATE(menuBarFactory())


class BaseWindow(_BaseWindowFields):
  """BaseWindow provides menus and actions for the main application
  window."""

  def __init__(self, *args, **kwargs) -> None:
    _BaseWindowFields.__init__(self, None)

  def initUI(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """
    # self.mainMenuBar.setupMenus()
    self.setMenuBar(self.mainMenuBar)

  def show(self) -> None:
    """Shows the window."""
    self.initUI()
    QMainWindow.show(self)
