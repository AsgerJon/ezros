"""App subclasses the QApplication class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from icecream import ic
from rospy import init_node

ic.configureOutput(includeContext=True)


class App(QApplication):
  """App is a subclass of QApplication."""

  icons = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
    self.setAttribute(Qt.ApplicationAttribute.AA_DontUseNativeMenuBar,
                      True)
    init_node('EZROS', anonymous=False)
