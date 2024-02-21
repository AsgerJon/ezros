"""The helpFactory creates a creator function for the help menu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow, QApplication
from vistutils.text import stringList

from ezros.gui.windows.icons import getIcon


def helpFactory() -> callable:
  """The helpFactory creates a creator function for the help menu ."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs) -> QMenu:
    """Creates the edit menu"""
    menu = QMenu('Edit', self)
    names = stringList("""About Qt, About Python, """)
    cutNames = stringList("""F11, F12""")
    cuts = [QKeySequence(cut) for cut in cutNames]
    tips = ["""Information about the Qt framework""",
            """Information about the Python programming language"""]
    icons = [getIcon(name) for name in ['about_qt', 'risitas']]
    for (name, cut, tip, icon) in zip(names, cuts, tips, icons):
      action = QAction()
      action.setText(name.capitalize().replace('_', ' '))
      action.setToolTip(tip)
      action.setShortcut(cut)
      action.setIcon(icon)
      action.setParent(self)
      if 'Qt' in name:
        setattr(self, 'aboutQtAction', action)
      else:
        setattr(self, 'aboutPythonAction', action)
      action.triggered.connect(QApplication.aboutQt)
      menu.addAction(action)
    return menu

  return callMeMaybe
