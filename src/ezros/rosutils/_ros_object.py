"""RosTopic class encapsulating ros topics."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import TYPE_CHECKING

from PySide6.QtCore import Signal, QCoreApplication, QObject

if TYPE_CHECKING:
  from ezside.app import App


class RosObject(QObject):
  """RosTopic class encapsulating ros topics."""

  echo = Signal(str)
  initQuit = Signal()

  def __init__(self, *args, **kwargs) -> None:
    for arg in args:
      if isinstance(arg, QObject):
        QObject.__init__(self, arg)
    else:
      QObject.__init__(self, )
    app = QCoreApplication.instance()
    if TYPE_CHECKING:
      assert isinstance(app, App)
    app.stopThreads.connect(self.initQuit)
