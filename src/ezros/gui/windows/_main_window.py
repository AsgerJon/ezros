"""MainWindow provides the main application window. The BaseWindow class
provides menus and actions. The Layout Window class provides the layout of
widget that appear on the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from ezros.gui.factories import timerFactory
from ezros.gui.windows import LayoutWindow
from morevistutils.fields import Field


class _MainWindowFields(LayoutWindow):
  """Fields for the MainWindow class."""

  paintTimer = Field(QTimer, )
  paintTimer.CREATE(timerFactory())


class MainWindow(_MainWindowFields):
  """MainWindow provides the main application window. The BaseWindow class
  provides menus and actions. The Layout Window class provides the layout of
  widget that appear on the main application window"""

  def __init__(self, *args, **kwargs) -> None:
    _MainWindowFields.__init__(self, *args, **kwargs)
    # self.aboutQtAction.triggered.connect(self.aboutQt)

  def aboutQt(self) -> None:
    """Displays the about Qt dialog."""
    QApplication.aboutQt()
