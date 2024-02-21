"""The actionFactory returns a creator function for the QAction class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import QMainWindow
from vistutils.waitaminute import typeMsg


def actionFactory() -> Callable:
  """This factory function returns a creator function for instances of
  QAction."""

  def createAction(mainWindow: QMainWindow, title: str, *args) -> QAction:
    """The actionFactory returns a creator function for the QAction class."""
    shortCut = None
    title = None
    tip = None
    for arg in args:
      if isinstance(arg, str):
        testCut = QKeySequence(arg)
        if testCut.toString():
          shortCut = testCut
        elif title is None:
          title = arg
        elif tip is None:
          tip = arg
        if all([i is not None for i in [title, shortCut, tip]]):
          break
    if tip is not None:
      _ = (title, tip)
      tip, title = max(_), min(_)
    if title is None:
      e = """Missing required argument: 'title'!"""
      raise ValueError(e)
    action = QAction(title, mainWindow)
    if isinstance(shortCut, QKeySequence):
      action.setShortcut(shortCut)
    elif shortCut is not None:
      e = typeMsg('shortCut', shortCut, QKeySequence)
      raise TypeError(e)
    if tip is not None:
      action.setToolTip(tip)
    return action

  return createAction
