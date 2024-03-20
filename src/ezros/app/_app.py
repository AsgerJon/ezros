"""App subclasses the QApplication class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication


class App(QApplication):
  """App is a subclass of QApplication."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
    self.setApplicationVersion('0.1.0')
    self.setOrganizationName('TMR')
    self.setOrganizationDomain('tmr.dk')
    self.setQuitOnLastWindowClosed(True)
    self.setWindowIcon(QIcon(':/icons/ezros.svg'))



