"""The filesFactory creates a creator function for the QMenu providing a
file menu for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow, QApplication
from vistutils.text import stringList

from ezros.gui.windows.icons import getIcon


def filesFactory() -> callable:
  """The filesFactory creates a creator function for the QMenu providing a
  file menu for the main application window."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs):
    """Creates the files menu"""
    menu = QMenu('File', self)
    names = stringList("""new, open, save, save_as, exit""")
    cutNames = stringList(
      """Ctrl+N, Ctrl+O, Ctrl+S, Ctrl+Shift+S, ALT+F4""")
    cuts = [QKeySequence(cut) for cut in cutNames]
    tips = ["""Create a new document""",
            """Open an existing document""",
            """Save changes to disk""",
            """Save changes to a new file""",
            """Exit the application"""]
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
