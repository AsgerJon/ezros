"""PubLayout provides the layout window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from ezros.gui.windows import BaseWindow
from _dep.morevistutils import Wait
from ezros.gui.widgets import BaseWidget, BaseLayout, LabelWidget


class PubLayoutWindow(BaseWindow):
  """The Pub class provides a gui control of a publisher in the ROS
  system."""

  baseWidget = Wait(BaseWidget)
  baseLayout = Wait(BaseLayout, )
  welcomeLabel = Wait(LabelWidget, 'Welcome', )
  label = Wait(LabelWidget, 'Goodbye')

  def initUI(self) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.welcomeLabel, 0, 0, 1, 1)
    self.baseLayout.addWidget(self.label, 1, 0, 1, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def connectActions(self) -> None:
    """Initializes the user interface"""
