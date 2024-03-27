"""The MainWindow class organizes the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QApplication
from icecream import ic

from ezros.app import LayoutWindow

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def initActions(self) -> None:
    self.menuBar.help.about_qt.triggered.connect(QApplication.aboutQt)
    self.menuBar.debug.debug1.triggered.connect(self.debug1func)

  def debug1func(self) -> None:
    """Debug 1 function"""
    ic(getattr(self.commandWidget, '_commander'))
