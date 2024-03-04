"""BaseWindow provides menus and actions for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow, \
  QWidget, \
  QMessageBox, \
  QStatusBar, \
  QMenu, QMenuBar
from icecream import ic

from ezros.gui.factories import menuBarFactory
from ezros.gui.shortnames import parseParent
from morevistutils import Wait

ic.configureOutput(includeContext=True)


class BaseWindow(QMainWindow):
  """BaseWindow provides menus and actions for the main application
  window."""

  mainMenuBar = Wait(menuBarFactory(), )

  mainMenuBar: QMenuBar

  filesMenu: QMenu
  editMenu: QMenu
  helpMenu: QMenu
  debugMenu: QMenu

  newAction: QAction
  openAction: QAction
  saveAction: QAction
  saveAsAction: QAction
  exitAction: QAction

  selectAllAction: QAction
  cutAction: QAction
  copyAction: QAction
  pasteAction: QAction
  undoAction: QAction
  redoAction: QAction

  aboutAction: QAction
  aboutQtAction: QAction
  aboutPythonAction: QAction

  debug01Action: QAction
  debug02Action: QAction
  debug03Action: QAction
  debug04Action: QAction
  debug05Action: QAction
  debug06Action: QAction
  debug07Action: QAction
  debug08Action: QAction
  debug09Action: QAction
  debug10Action: QAction

  def __init__(self, *args, **kwargs) -> None:
    self.__owned_actions__ = {}
    parent = parseParent(*args, )
    QMainWindow.__init__(self, parent, )

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

  def initUI(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """

  def connectActions(self) -> None:
    """Connects the actions to the slots. Subclasses are required to provide
    implementation of this method. """

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
