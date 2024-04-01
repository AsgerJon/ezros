"""CommandControl widget class implementation providing control of a topic
expecting AuxCommand type."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.core import Precise
from ezside.widgets import BaseWidget
from icecream import ic
from vistutils.waitaminute import typeMsg

from ezros.defaults import Settings
from ezros.widgets import PushButton, Vertical, VerticalSpacer, Label
from ezros.widgets import HorizontalSeparator, AbstractCommandWidget

ic.configureOutput(includeContext=True)


class CommandControl(AbstractCommandWidget):
  """CommandControl widget class implementation providing control of a topic
  expecting AuxCommand type."""

  baseLayout = AttriBox[Vertical]()
  topicLabel = AttriBox[Label]()
  h1 = AttriBox[HorizontalSeparator]()
  activateButton = AttriBox[PushButton]()
  deactivateButton = AttriBox[PushButton]()
  holdButton = AttriBox[PushButton]()
  h2 = AttriBox[HorizontalSeparator]()
  s = AttriBox[VerticalSpacer]()

  def __init__(self, name: str) -> None:
    """Initialize the widget."""
    AbstractCommandWidget.__init__(self, )
    if name is None:
      e = """Required argument 'name' not provided!"""
      raise ValueError(e)
    if not isinstance(name, str):
      e = typeMsg('name', name, str)
      raise TypeError(e)
    setattr(self, '__topic_name__', name)

  def getTopicName(self) -> str:
    """Get the topic name."""
    name = getattr(self, '__topic_name__', None)
    if name is None:
      e = """The topic name has not been set!"""
      raise ValueError(e)
    if isinstance(name, str):
      return name
    e = typeMsg('name', name, str)
    raise TypeError(e)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.activeTimer.setInterval(500)
    self.activeTimer.setTimerType(Precise)
    self.activeTimer.setSingleShot(False)

    self.baseLayout.addWidget(self.h1)

    self.topicLabel.text = self.getTopicName()
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

    self.baseLayout.addWidget(self.h2)

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
