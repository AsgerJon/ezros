"""LivePlot shows a live plot of the data in the given data source."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QVBoxLayout
from attribox import AttriBox, this
from ezside.core import Precise
from ezside.widgets import DataView, TextLabel
from genpy import Message
from icecream import ic
from rospy import Subscriber, init_node
from vistutils.waitaminute import typeMsg

from ezros.rosutils import Sub, resolveTopicType
from ezros.widgets import TabPage

ic.configureOutput(includeContext=True)


class LivePlot(TabPage):
  """LivePlot shows a live plot of the data in the given data source."""

  welcomeBanner = AttriBox[TextLabel]('Welcome to ez-ROS!')
  dataView = AttriBox[DataView]()
  baseLayout = AttriBox[QVBoxLayout]()
  timer = AttriBox[QTimer](20, Precise, singleShot=False)

  def __init__(self, *args, **kwargs) -> None:
    TabPage.__init__(self, *args, **kwargs)
    init_node('ezros')
    self.sub = Sub('yolo')

  def initUi(self) -> None:
    """Initialize the user interface."""
    ic()
    self.sub.setCallback(self.preAppend)
    self.welcomeBanner.initUi()
    self.baseLayout.addWidget(self.welcomeBanner)
    self.dataView.initUi()
    self.baseLayout.addWidget(self.dataView)
    self.setLayout(self.baseLayout)

  def connectActions(self) -> None:
    """Connect actions to slots."""
    self.timer.timeout.connect(self.refresh)
    self.timer.start()

  def refresh(self) -> None:
    """Refresh the data."""
    self.dataView.refresh()

  def preAppend(self, message: Message) -> None:
    """Prepend a message to the data."""
    ic(message)
    value = getattr(message, 'data', None)
    if value is None:
      return
    if isinstance(value, (int, float)):
      return self.append(value)
    e = typeMsg('value', value, float)
    raise TypeError(e)

  def append(self, value: float) -> None:
    """Append a value to the data."""
    self.dataView.append(value)

  def getTopicName(self) -> str:
    """Getter-function for the currently defined topic name"""
    return 'yolo'
