"""TalkApp provides a subclass of QApplication containing the application
itself."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from rospy import init_node

# from ezros.rosutils import initNodeMaybe

MenuFlag = Qt.ApplicationAttribute.AA_DontUseNativeMenuBar


class TalkApp(QApplication):
  """TalkApp provides a subclass of QApplication containing the application
  itself."""

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the application."""
    QApplication.__init__(self, *args, **kwargs)
    self.setApplicationName('Talker')
    self.setApplicationDisplayName('Talker')
    self.setAttribute(MenuFlag, True)
    init_node('talker', anonymous=False)
