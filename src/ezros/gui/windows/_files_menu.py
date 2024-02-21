"""FilesMenu provides the file menu for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow

from morevistutils.fields import Field

from ezros.gui.windows import AbstractMenu


class _FilesMenuFields(AbstractMenu):
  """Convenience class containing fields for FilesMenu."""
  newAction = Field(QAction, 'New', 'Ctrl+N',
                    'Create a new file')
  openAction = Field(QAction, 'Open', 'Ctrl+O',
                     'Open an existing file')
  saveAction = Field(QAction, 'Save', 'Ctrl+S',
                     'Save the current file')
  saveAsAction = Field(QAction, 'Save As...', 'Ctrl+Shift+S',
                       'Save the current file under a new name')
  exitAction = Field(QAction, 'Exit', 'Ctrl+Q', 'Exit the application')
  newAction.CREATE(AbstractMenu.createAction)
  openAction.CREATE(AbstractMenu.createAction)
  saveAction.CREATE(AbstractMenu.createAction)
  saveAsAction.CREATE(AbstractMenu.createAction)
  exitAction.CREATE(AbstractMenu.createAction)


class FilesMenu(_FilesMenuFields):
  """FilesMenu provides the file menu for the main application window."""

  def __init__(self, parent: QMainWindow, title: str) -> None:
    _FilesMenuFields.__init__(self, parent)
    
  def setupMenu(self) -> None:
    """Sets up the menu for the window."""
    self.addAction(self.newAction)
    self.addAction(self.openAction)
    self.addAction(self.saveAction)
    self.addAction(self.saveAsAction)
    self.addSeparator()
    self.addAction(self.exitAction)
