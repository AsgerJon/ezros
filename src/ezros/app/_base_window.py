"""BaseWindow class provides menus and actions for the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QMainWindow
from attribox import AttriBox, this
from ezside import MenuBar, StatusBar


class BaseWindow(QMainWindow):
  """BaseWindow class provides menus and actions for the application."""

  menuBar = AttriBox[MenuBar](this)
  statusBar = AttriBox[StatusBar](this)

  def initMenus(self) -> None:
    """Initialize the menus."""
    self.menuBar.initUi()
    self.setMenuBar(self.menuBar)
    self.statusBar.initUi()
    self.setStatusBar(self.statusBar)

  @abstractmethod
  def initUi(self) -> None:
    """Initialize the user interface."""

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""

  def show(self) -> None:
    """Show the window."""
    self.initMenus()
    self.initUi()
    self.initActions()
    QMainWindow.show(self)
