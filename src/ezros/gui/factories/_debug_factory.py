"""The debugFactory returns a creator for the debug menu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.gui.factories.icons import getIcon


def debugFactory(num: int = None) -> Callable:
  """The debugFactory returns a creator for the debug menu."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs) -> QMenu:
    """Creates the edit menu"""
    menu = QMenu('Debug', self)
    numDebug = maybe(num, 10)
    names = ['** Debug %02d **' % i for i in range(1, numDebug)]
    keys = ['debug%02dAction' % i for i in range(1, numDebug)]
    cuts = [QKeySequence('F%d' % (i)) for i in range(1, numDebug)]
    tips = ['Debug %02d' % i for i in range(1, numDebug)]
    icons = [getIcon('risitas') for i in range(1, numDebug)]
    for (key, name, cut, tip, icon) in zip(keys, names, cuts, tips, icons):
      action = self.addAction(name, key, )
      if not isinstance(action, QAction):
        e = typeMsg('action', action, QAction)
        raise TypeError(e)
      action.setToolTip(tip)
      action.setShortcut(cut)
      action.setIcon(icon)
      action.setParent(self)
      menu.addAction(action)
    return menu

  return callMeMaybe
