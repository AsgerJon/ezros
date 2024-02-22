"""The filesFactory creates a creator function for the QMenu providing a
file menu for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QKeySequence, QAction
from PySide6.QtWidgets import QMenu, QMainWindow
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.gui.windows.icons import getIcon


def filesFactory() -> callable:
  """The filesFactory creates a creator function for the QMenu providing a
  file menu for the main application window."""

  def callMeMaybe(self: QMainWindow, *args, **kwargs):
    """Creates the files menu"""
    menu = QMenu('File', self)
    names = stringList("""New, Open, Save, Save As, EXIT""")
    keys = ['%s%s' % (name[0].lower(), name[1:]) for name in names]
    keys = [key.replace(' ', '') for key in keys]
    cutNames = stringList(
      """Ctrl+N, Ctrl+O, Ctrl+S, Ctrl+Shift+S, ALT+F4""")
    cuts = [QKeySequence(cut) for cut in cutNames]
    tips = ["""Create a new document""",
            """Open an existing document""",
            """Save changes to disk""",
            """Save changes to a new file""",
            """Exit the application"""]
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
