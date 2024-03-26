"""RosTalker is a widget for publishing messages to a ROS topic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from warnings import warn

from attribox import AttriBox
from ezside.widgets import BaseWidget, TextLabel
from genpy import Time
from msgs.msg import Float32Stamped
from rospy import Publisher, Subscriber
from std_msgs.msg import Header
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
from ezros.widgets import PushButton, LineEdit, SpinBox, Grid, TightLabel


class RosTalker(BaseWidget):
  """RosTalker is a widget for publishing messages to a ROS topic."""

  __inner_publisher__ = None
  __inner_subscriber__ = None

  baseLayout = AttriBox[Grid]()
  sayButton = AttriBox[PushButton]('Say')
  node = AttriBox[LineEdit]('Node name')
  topic = AttriBox[LineEdit]('Topic name')
  spinBox = AttriBox[SpinBox]('data', 'horizontal', 0, 50, 100)
  textLabel = AttriBox[TightLabel]('**waiting for publisher**')

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.topic)
    self.baseLayout.addWidget(self.spinBox)
    self.baseLayout.addWidget(self.sayButton)
    self.baseLayout.addWidget(self.textLabel)
    self.setLayout(self.baseLayout)
    self.initActions()

  def initActions(self) -> None:
    """Initialize the actions."""
    self.sayButton.clicked.connect(self.publish)

  def createPublisher(self) -> None:
    """Create a publisher."""
    if self.__inner_publisher__ is None:
      name = self.topic.text()
      type_ = resolveTopicType(name)
      self.__inner_publisher__ = Publisher(name, type_)
    else:
      w = """Publisher already exists."""
      warn(w)

  def getPublisher(self, **kwargs) -> Publisher:
    """Get the publisher."""
    if self.__inner_publisher__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createPublisher()
      return self.getPublisher(_recursion=True)
    if isinstance(self.__inner_publisher__, Publisher):
      return self.__inner_publisher__
    name = 'self.__inner_publisher__'
    value = self.__inner_publisher__
    expected = Publisher
    e = typeMsg(name, value, expected)
    raise TypeError(e)

  def publish(self, ) -> None:
    """Publish a message."""
    message = Float32Stamped()
    message.data = self.spinBox.inner.value()
    header = Header()
    header.stamp = Time.from_sec(time.time())
    header.frame_id = 'talker'
    message.header = header
    self.getPublisher().publish(message)

  def callback(self, message: Float32Stamped) -> None:
    """Callback function."""
    data = message.data
    txt = 'Received: %.12E' % data
    self.textLabel.setText(txt)

  def createSubscriber(self) -> None:
    """Create a subscriber."""
    if self.__inner_subscriber__ is None:
      name = self.topic.text()
      type_ = resolveTopicType(name)
      self.__inner_subscriber__ = Subscriber(name, type_, self.callback)
    else:
      w = """Subscriber already exists."""
      warn(w)

  def getSubscriber(self, **kwargs) -> Subscriber:
    """Get the subscriber."""
    if self.__inner_subscriber__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createSubscriber()
      return self.getSubscriber(_recursion=True)
    if isinstance(self.__inner_subscriber__, Subscriber):
      return self.__inner_subscriber__
    name = 'self.__inner_subscriber__'
    value = self.__inner_subscriber__
    expected = Subscriber
    e = typeMsg(name, value, expected)
    raise TypeError(e)
