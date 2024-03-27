"""CommandControl widget class implementation providing control of a topic
expecting AuxCommand type."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.core import Precise
from ezside.widgets import BaseWidget
from icecream import ic

from ezros.defaults import Settings
from ezros.widgets import PushButton, Vertical, VerticalSpacer, Label
from ezros.widgets import HorizontalSeparator, AbstractCommandWidget

ic.configureOutput(includeContext=True)


class CommandControl(AbstractCommandWidget):
  """CommandControl widget class implementation providing control of a topic
  expecting AuxCommand type."""

  baseLayout = AttriBox[Vertical]()
  topicLabel = AttriBox[Label]()
  separator = AttriBox[HorizontalSeparator]()
  activateButton = AttriBox[PushButton]()
  deactivateButton = AttriBox[PushButton]()
  holdButton = AttriBox[PushButton]()
  s = AttriBox[VerticalSpacer]()

  def __init__(self, name: str) -> None:
    """Initialize the widget."""
    AbstractCommandWidget.__init__(self, )
    self._topicName = name

  def getTopicName(self) -> str:
    """Get the topic name."""
    return self._topicName

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.activeTimer.setInterval(500)
    self.activeTimer.setTimerType(Precise)
    self.activeTimer.setSingleShot(False)

    self.topicLabel.text = self._topicName
    self.topicLabel.setFont(Settings.getHeaderFont())
    self.baseLayout.addWidget(self.topicLabel)

    self.activateButton.setText('Activate')
    self.activateButton.setFont(Settings.getButtonFont())
    self.baseLayout.addWidget(self.activateButton)

    self.deactivateButton.setText('Deactivate')
    self.deactivateButton.setFont(Settings.getButtonFont())
    self.baseLayout.addWidget(self.deactivateButton)

    self.holdButton.setText('Hold')
    self.holdButton.setFont(Settings.getButtonFont())
    self.baseLayout.addWidget(self.holdButton)

    self.baseLayout.addWidget(self.s)

    self.setLayout(self.baseLayout)
    BaseWidget.initUi(self)
    self.setMinimumWidth(144)

  def connectActions(self) -> None:
    """Initialize the actions."""
    AbstractCommandWidget.connectActions(self)
    self.activateButton.clicked.connect(self.start)
    self.deactivateButton.clicked.connect(self.stop)
    self.holdButton.holdOn.connect(self.start)
    self.holdButton.holdOff.connect(self.stop)
