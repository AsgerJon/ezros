"""BaseWindow provides menus and actions for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow, QWidget, QMessageBox, QStatusBar
from icecream import ic

from ezros.gui.factories import menuBarFactory
from morevistutils import Wait

ic.configureOutput(includeContext=True)


class BaseWindow(QMainWindow):
  """BaseWindow provides menus and actions for the main application
  window."""

  mainMenuBar = Wait(menuBarFactory(), )

  def __init__(self, *args, **kwargs) -> None:
    self.__owned_actions__ = {}
    for arg in args:
      if isinstance(arg, QWidget):
        QMainWindow.__init__(self, arg)
        break
    else:
      QMainWindow.__init__(self, )

  def addAction(self, *args) -> QAction:
    """Adds an action to the window."""
    action = QMainWindow.addAction(self, *args)
    name, key = [*args, None, None][:2]
    if isinstance(name, str) and isinstance(key, str):
      if key in self.__owned_actions__:
        e = """Action with key '%s' already exists. """
        raise KeyError(e % key)
      self.__owned_actions__[key] = action
      setattr(self, key, action)
    else:
      e = """Both name and key are required arguments!"""
      raise ValueError(e)
    return action

  def getOwnedActions(self) -> dict:
    """Returns the owned actions."""
    return self.__owned_actions__

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """

  @abstractmethod
  def connectActions(self) -> None:
    """Connects the actions to the slots. Subclasses are required to provide
    implementation of this method. """

  @abstractmethod
  def createActionStub(self) -> None:
    """Creates a stub file. Subclasses are required to provide
    implementation of this method. """

  def show(self) -> None:
    """Shows the window."""
    self.setMenuBar(self.mainMenuBar)
    self.setStatusBar(QStatusBar(self))
    self.initUI()
    self.connectActions()
    self.createActionStub()
    QMainWindow.show(self)

  def aboutPython(self) -> None:
    """Displays the about Python dialog."""
    x, y, z = [sys.version_info.f for f in ['major', 'minor', 'micro']]
    QMessageBox.about(self, 'Python %d.%d.%d' % (x, y, z),
                      'Python %d.%d.%d' % (x, y, z))
