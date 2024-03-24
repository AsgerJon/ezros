"""LayoutWindow organizes the layouts used in the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QVBoxLayout
from attribox import AttriBox
from ezside import BaseWindow
from ezside.widgets import BaseWidget, TextLabel

from ezros.widgets import Button


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[QVBoxLayout]()
  welcomeBanner = AttriBox[TextLabel]('Welcome to ez-ROS!')
  testButton = AttriBox[Button]('Test Button')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.welcomeBanner.initUi()
    self.baseLayout.addWidget(self.welcomeBanner)
    self.testButton.initUi()
    self.baseLayout.addWidget(self.testButton)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
    self.connectActions()

  def show(self) -> None:
    """Show the window."""
    self.initUi()
    BaseWindow.show(self)
