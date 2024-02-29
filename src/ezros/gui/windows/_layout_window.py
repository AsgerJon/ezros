"""LayoutWindow class for the layout window of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QPushButton, QComboBox
from ezros.gui.widgets import LabelWidget, BaseWidget, BaseLayout, \
  DataWidget, PromptWidget, SafetyToggleButton, JKFlipFlop, RetroPushButton
from ezros.gui.windows import BaseWindow
from morevistutils import Wait


class LayoutWindow(BaseWindow):
  """LayoutWindow class for the layout window of the application."""

  baseWidget = Wait(BaseWidget)
  baseLayout = Wait(BaseLayout, )
  welcomeLabel = Wait(LabelWidget, 'Welcome', )
  # state = Wait(PromptWidget, 'YOLO', )
  goodbyeLabel = Wait(LabelWidget, 'Goodbye')
  data = Wait(DataWidget, )
  toggle = Wait(SafetyToggleButton, )

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.testTimer = None
    self._activateButton = QPushButton('Pump ON')
    self._disableButton = QPushButton('Pump OFF')
    self._pumpComboBox = QComboBox()
    self._sprayComboBox = QComboBox()
    self.setMinimumSize(480, 320)

  def initUI(self) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self._pumpComboBox, 0, 0, 1, 1)
    # self.baseLayout.addWidget(self.state, 0, 1, 1, 1)
    self.baseLayout.addWidget(self.toggle, 0, 1, 1, 1, )
    self.baseLayout.addWidget(self._sprayComboBox, 0, 2, 1, 1)
    self.baseLayout.addWidget(self.data, 1, 0, 1, 3)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  @abstractmethod
  def connectActions(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """
