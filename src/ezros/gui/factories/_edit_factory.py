"""The editFactory returns a creator function for the QLineEdit class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow
from icecream import ic
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.gui.factories.icons import getIcon

ic.configureOutput(includeContext=True)


def editFactory() -> callable:
  """This factory function returns a creator function for instances of
  QLineEdit."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs) -> QMenu:
    """Creates the edit menu"""
    menu = QMenu('Edit', self)
    names = stringList("""Undo, Redo, Cut, Copy, Paste, Delete, 
    Select All""")
    keys = ['%s%s' % (name[0].lower(), name[1:]) for name in names]
    keys = [key.replace(' ', '') for key in keys]
    cutNames = stringList(
      """Ctrl+Z, Ctrl+Y, Ctrl+X, Ctrl+C, Ctrl+V, Del, Ctrl+A""")
    cuts = [QKeySequence(cut) for cut in cutNames]
    tips = ["""Undo the last action""",
            """Redo the previously undone action""",
            """Cut the selection and copy it to the clipboard""",
            """Copy the selection to the clipboard""",
            """Paste from the clipboard""",
            """Delete the selected item""",
            """Select all items in the document"""]
    icons = [getIcon(name) for name in names]
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
