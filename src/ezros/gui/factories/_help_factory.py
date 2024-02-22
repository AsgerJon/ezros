"""The helpFactory creates a creator function for the help menu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.gui.windows.icons import getIcon


def helpFactory() -> callable:
  """The helpFactory creates a creator function for the help menu ."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs) -> QMenu:
    """Creates the edit menu"""
    menu = QMenu('Edit', self)
    names = stringList("""About Qt, About Python, """)
    keys = stringList("""aboutQtAction, aboutPythonAction""")
    cutNames = stringList("""F11, F12""")
    cuts = [QKeySequence(cut) for cut in cutNames]
    tips = ["""Information about the Qt framework""",
            """Information about the Python programming language"""]
    icons = [getIcon(name) for name in ['about_qt', 'risitas']]
    for (key, name, cut, tip, icon) in zip(keys, names, cuts, tips, icons):
      action = self.addAction(name)
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
