"""PubLayout provides the layout window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QLineEdit, QLabel, QWidget, QGridLayout, \
  QPushButton

from ezros.gui.widgets import BaseWidget, BaseLayout, LabelWidget, \
  BaseLineEdit
from ezros.gui.windows import BaseWindow
from ezros.rosutils import validateInitialized
from morevistutils import Wait


class PubLayoutWindow(BaseWindow):
  """The Pub class provides a gui control of a publisher in the ROS
  system."""

  baseWidget = Wait(BaseWidget)
  baseLayout = Wait(BaseLayout, )
  welcomeLabel = Wait(LabelWidget, 'Welcome', )
  noteName = Wait(BaseLineEdit, 'Input note name here...', )
  noteInitButton = Wait(QPushButton, 'Initialize note', )
  topicName = Wait(BaseLineEdit, 'Input topic name here...', )

  def initUI(self) -> None:
    """Sets up the widgets"""
    welcomeLabel = Wait(LabelWidget, 'Welcome', )
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
    BaseWindow.initUI(self)

  @abstractmethod
  def connectActions(self) -> None:
    """Initializes the user interface"""
