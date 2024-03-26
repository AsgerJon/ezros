"""LayoutWindow organizes the layouts used in the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from attribox import AttriBox
from ezside.widgets import BaseWidget, TextLabel, PushButton
from icecream import ic

from ezros.app import BaseWindow
from ezros.widgets import Vertical, RosTalker

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Vertical]()
  welcomeLabel = AttriBox[TextLabel]('Welcome to EZROS')
  rosTalker = AttriBox[RosTalker]()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.welcomeLabel)
    self.baseLayout.addWidget(self.rosTalker)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def initActions(self) -> None:
    """Initialize the actions."""
