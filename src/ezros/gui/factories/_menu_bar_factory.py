"""The menuBarFactory returns a creator function for the menu bar taking a
any number of QMenu instances as arguments."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtWidgets import QMainWindow, QMenuBar
from icecream import ic

from ezros.gui.factories import filesFactory, editFactory, helpFactory, \
  debugFactory


def menuBarFactory() -> Callable:
  """This factory function returns a creator function for instances of
  QMenuBar."""

  def createMenuBar(mainWindow: QMainWindow, *args) -> QMenuBar:
    """The menuBarFactory returns a creator function for the menu bar
    taking any number of QMenu instances as arguments."""
    bar = QMenuBar(mainWindow)
    bar.addMenu(filesFactory()(mainWindow))
    bar.addMenu(editFactory()(mainWindow))
    bar.addMenu(helpFactory()(mainWindow))
    bar.addMenu(debugFactory(10)(mainWindow))
    return bar

  return createMenuBar
