"""MenuBar provides the menu bar for the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMenuBar

from ezros.gui.windows import FilesMenu
from morevistutils.fields import Field


class _MenuBarFields(QMenuBar):
  """Convenience class containing fields for MenuBar."""

  files = Field(FilesMenu)
  # edit = Field(EditMenu)
  # view = Field(ViewMenu)
  # help = Field(HelpMenu)


class MenuBar(_MenuBarFields):
  """MenuBar provides the menu bar for the main application window."""

  def __init__(self, *args, **kwargs) -> None:
    _MenuBarFields.__init__(self, None)

  def setupMenus(self) -> None:
    """Sets up the menus for the window."""
    self.files.setupMenu()
    self.addMenu(self.files)
  #  self.edit.setupMenu()
  #  self.addMenu(self.edit)
  #  self.view.setupMenu()
  #  self.addMenu(self.view)
  #  self.help.setupMenu()
  #  self.addMenu(self.help)
  #
