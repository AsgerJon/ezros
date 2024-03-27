"""CommandControl widget class implementation providing control of a topic
expecting AuxCommand type."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.core import Precise
from ezside.widgets import BaseWidget
from icecream import ic

from ezros.rosutils import EmptyField
from ezros.widgets import AbstractCommandWidget, \
  PushButton, \
  LineEdit, \
  Vertical

ic.configureOutput(includeContext=True)


class CommandControl(AbstractCommandWidget):
  """CommandControl widget class implementation providing control of a topic
  expecting AuxCommand type."""

  __topic_name__ = EmptyField()

  baseLayout = AttriBox[Vertical]()
  topicInput = AttriBox[LineEdit]()
  newTopicButton = AttriBox[PushButton]()
  activateButton = AttriBox[PushButton]()
  deactivateButton = AttriBox[PushButton]()
  holdButton = AttriBox[PushButton]()

  @__topic_name__.GET
  def getTopicName(self) -> str:
    """Get the topic name."""
    return self.topicInput.text

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.activeTimer.setInterval(500)
    self.activeTimer.setTimerType(Precise)
    self.activeTimer.setSingleShot(False)
    self.topicInput.setPlaceholderText('Enter topic name')
    self.newTopicButton.setText('Apply')
    self.activateButton.setText('Activate')
    self.deactivateButton.setText('Deactivate')
    self.holdButton.setText('Hold')
    self.baseLayout.addWidget(self.topicInput)
    self.baseLayout.addWidget(self.newTopicButton)
    self.baseLayout.addWidget(self.activateButton)
    self.baseLayout.addWidget(self.deactivateButton)
    self.baseLayout.addWidget(self.holdButton)
    self.setLayout(self.baseLayout)
    BaseWidget.initUi(self)

  def connectActions(self) -> None:
    """Initialize the actions."""
    AbstractCommandWidget.connectActions(self)
    self.activateButton.clicked.connect(self.start)
    self.activateButton.clicked.connect(self.startTest)
    self.deactivateButton.clicked.connect(self.stop)
    self.deactivateButton.clicked.connect(self.stopTest)

  def stopTest(self, ) -> None:
    """Stop the test"""
    ic('stop')

  def startTest(self, ) -> None:
    """Start the test"""
    ic('start')
