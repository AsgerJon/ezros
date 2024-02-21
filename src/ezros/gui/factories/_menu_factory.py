"""The menuFactory returns a creator function for the QMenu class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow, QMenu
from vistutils.waitaminute import typeMsg


def menuFactory() -> Callable:
  """This factory function returns a creator function for instances of
  QMenu."""

  def createMenu(mainWindow: QMainWindow, title: str, *actions) -> QMenu:
    """The menuFactory returns a creator function for the QMenu class."""
    if isinstance(mainWindow, QMainWindow):
      menu = QMenu(title, mainWindow)
    elif mainWindow is None:
      menu = QMenu(title)
    else:
      e = typeMsg('mainWindow', mainWindow, QMainWindow)
      raise TypeError(e)
    for action in actions:
      if isinstance(action, QAction):
        menu.addAction(action)
    return menu

  return createMenu
