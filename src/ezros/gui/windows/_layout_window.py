"""LayoutWindow class for the layout window of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from ezros.gui.widgets import LabelWidget, BaseWidget, BaseLayout
from ezros.gui.windows import BaseWindow
from ezros.morevistutils.fields import Later


class LayoutWindow(BaseWindow):
  """LayoutWindow class for the layout window of the application."""

  baseWidget = Later(BaseWidget)
  baseLayout = Later(BaseLayout, )
  welcomeLabel = Later(LabelWidget, 'Welcome', )
  goodbyeLabel = Later(LabelWidget, 'Goodbye', )

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.setMinimumSize(480, 320)

  def initUI(self) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.welcomeLabel, 0, 0)
    self.baseLayout.addWidget(self.goodbyeLabel, 1, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def connectActions(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """
