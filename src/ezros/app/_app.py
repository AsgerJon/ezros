"""App subclasses the QApplication class."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from ezros.rosutils import initNodeMaybe

MenuFlag = Qt.ApplicationAttribute.AA_DontUseNativeMenuBar


class App(QApplication):
  """App is a subclass of QApplication."""

  __caller_id__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    print('App.__init__')
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
    self.setAttribute(MenuFlag, True)
    self.__caller_id__ = initNodeMaybe('EZROS', anonymous=False,
                                       xmlrpc_port=33333, tcpros_port=33334)
