"""AbstractMenu provides the functionality shared by all menus."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Self, Any

from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import QMenu
from vistutils.waitaminute import typeMsg

from ezros.gui.windows.icons import getIcon


class AbstractMenu(QMenu):
  """AbstractMenu provides the functionality shared by all menus."""

  __iter_contents__ = None

  def createAction(self, cls: type, *args, **kwargs) -> QAction:
    """Creates a new QAction."""
    strArgs = [arg for arg in args if isinstance(arg, str)]
    if len(strArgs) < 3:
      e = """QAction requires at least three string arguments: title, 
       shortcut, and status_tip. """
      raise ValueError(e)
    title, shortcut, statusTip = strArgs[:3]
    icon = getIcon(title)
    shortCut = QKeySequence(shortcut)
    action = QAction()
    action.setText(title)
    action.setToolTip(statusTip)
    action.setShortcut(shortCut)
    action.setIcon(icon)
    action.setParent(self)
    return action

  def __iter__(self, ) -> Self:
    """Returns an iterator over the contents of the menu."""
    if hasattr(self, 'fieldRegistry'):
      fields = getattr(self, 'fieldRegistry', {})
      if isinstance(fields, dict):
        self.__iter_contents__ = fields.get(QAction, [])
      else:
        e = typeMsg('fields', fields, dict)
        raise TypeError(e)
    else:
      setattr(self, 'fieldRegistry', {})
      self.__iter_contents__ = []
    return self

  def __next__(self, ) -> Any:
    """Implementation of the iterator."""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration
