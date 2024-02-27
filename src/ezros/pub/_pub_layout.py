"""PubLayout provides the layout window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QLineEdit, QLabel, QWidget, QGridLayout, \
  QPushButton

from ezros.gui.windows import LayoutWindow
from ezros.rosutils import validateInitialized


class PubLayoutWindow(LayoutWindow):
  """The Pub class provides a gui control of a publisher in the ROS
  system."""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self.nodeLabel = QLabel('Node:')
    self.nodeLine = QLineEdit()
    self.initButton = QPushButton('Init Note')
    self.noteStatus = QLabel('Note Status:')
    self.baseLayout = QGridLayout()
    self.baseWidget = QWidget()

  def initUI(self) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.nodeLabel, 0, 0)
    self.baseLayout.addWidget(self.nodeLine, 0, 1)

    LayoutWindow.initUI(self)

  def _updateStatus(self, msg: str) -> None:
    """Updates the status of the note"""
    self.noteStatus.setText(msg)

  def _initNote(self, ) -> None:
    """Initializes the note"""
    noteName = self.nodeLine.text()
    msg = """Attempting to initialize note at name: %s""" % noteName
    self._updateStatus(msg)
    validateInitialized()

  @abstractmethod
  def connectActions(self) -> None:
    """Initializes the user interface"""
