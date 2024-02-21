"""The editFactory returns a creator function for the QLineEdit class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow, QApplication
from vistutils.text import stringList

from ezros.gui.windows.icons import getIcon


def editFactory() -> callable:
  """This factory function returns a creator function for instances of
  QLineEdit."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs) -> QMenu:
    """Creates the edit menu"""
    menu = QMenu('Edit', self)
    names = stringList("""undo, redo, cut, copy, paste, delete, 
    select_all""")
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
    for (name, cut, tip, icon) in zip(names, cuts, tips, icons):
      action = QAction()
      action.setText(name.capitalize().replace('_', ' '))
      action.setToolTip(tip)
      action.setShortcut(cut)
      action.setIcon(icon)
      action.setParent(self)
      setattr(self, '%sAction' % name, action)
      action.triggered.connect(QApplication.aboutQt)
      menu.addAction(action)
    return menu

  return callMeMaybe
