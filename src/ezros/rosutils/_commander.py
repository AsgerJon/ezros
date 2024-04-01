"""Commander issues instances of AuxCommand"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional, Self

from ezside.core import Precise
from ezside.widgets import BaseWidget
from icecream import ic
from msgs.msg import AuxCommand
from rospy import init_node, Publisher, ROSException, Time, Duration
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.rosutils import initNodeMaybe

ic.configureOutput(includeContext=True)


class Commander:
  """Commander issues instances of AuxCommand"""

  __timer_interval__ = 500
  __timer_type__ = Precise
  __single_shot__ = False

  __fallback_node_name__ = 'Test'
  __fallback_topic_name__ = 'Test'

  __node_name__ = None
  __topic_name__ = None
  __inner_publisher__ = None

  def initiateNode(self, ) -> None:
    """Initiates the node using class specific node name"""
    initNodeMaybe(self.__node_name__, anonymous=False)

  @classmethod
  def _reset(cls, self: Self, topicName: str) -> Self:
    """Replaces the given commander with a new one at a new topic name"""
    ic(self.__topic_name__, topicName)
    node = getattr(self, '__node_name__', cls.__fallback_node_name__)

    return cls(topicName, node)

  @staticmethod
  def typeGuard(node: str, topic: str) -> tuple[str, str]:
    """Type guard for the node and topic."""
    if not isinstance(node, str):
      e = typeMsg('node', node, str)
      raise TypeError(e)
    if not isinstance(topic, str):
      e = typeMsg('topic', topic, str)
      raise TypeError(e)
    return node, topic

  @classmethod
  def parseArgs(cls, *args, **kwargs) -> tuple[str, str]:
    """Parses the argument. This method is used when the constructor does
    not receive a BaseWidget instance. """
    nodeKwarg, nodeArg, nodeDefault = None, None, cls.__fallback_node_name__
    topicKwarg, topicArg, topicDefault = (None, None,
                                          cls.__fallback_topic_name__)
    keys = stringList("""node, topic""")
    strArgs = [arg for arg in args if isinstance(arg, str)]
    topicKwarg = kwargs.get('topic', None)
    nodeKwarg = kwargs.get('node', None)
    topicArg, nodeArg = [*strArgs, None, None][:2]
    topic = maybe(topicKwarg, topicArg, topicDefault)
    node = maybe(nodeKwarg, nodeArg, nodeDefault)
    return cls.typeGuard(node, topic)

  def parseThis(self, *args, **kwargs) -> Optional[tuple[str, str]]:
    """Attempts to extract a base widget from the arguments, which is then
    responsible for providing the topic and node. """
    for arg in args:
      if isinstance(arg, BaseWidget):
        widget = arg
        break
    else:
      return
    topic = getattr(widget, '__topic_name__', None)
    if topic is None:
      e = """The widget does not have a topic name."""
      raise AttributeError(e)
    node = maybe(getattr(widget, '__node_name__', None),
                 self.__fallback_node_name__)
    if isinstance(topic, str) and isinstance(node, str):
      return self.typeGuard(node, topic)

  def __init__(self, *args, **kwargs) -> None:
    try:
      node, topic = self.parseThis(*args, **kwargs)
    except TypeError as typeError:
      try:
        node, topic = self.parseArgs(*args, **kwargs)
      except TypeError as typeError2:
        raise typeError from typeError2
    self.__topic_name__ = topic
    self.__node_name__ = node
    self.initiateNode()

  def reset(self, topicName: str) -> Self:
    """Reset the commander."""
    if topicName == self.__topic_name__:
      return self
    return self._reset(self, topicName)

  def createPublisher(self, ) -> None:
    """Create the publisher."""
    ic(self.__topic_name__)
    self.initiateNode()
    publisher = Publisher(self.__topic_name__, AuxCommand, queue_size=10)
    setattr(self, '__inner_publisher__', publisher)

  def getPublisher(self, **kwargs) -> Publisher:
    """Return the publisher."""
    if not self.__inner_publisher__:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createPublisher()
      return self.getPublisher(_recursion=True)
    if isinstance(self.__inner_publisher__, Publisher):
      return self.__inner_publisher__
    name = 'self.__inner_publisher__'
    value = self.__inner_publisher__
    e = typeMsg(name, value, Publisher)
    raise TypeError(e)

  def __str__(self, ) -> str:
    """Return the string representation of the Commander."""
    return '%s - %s' % (self.__node_name__, self.__topic_name__)

  def publish(self, command: AuxCommand = None, **kwargs) -> None:
    """Publish a command."""
    try:
      self.getPublisher().publish(command)
    except ROSException as rosException:
      if kwargs.get('_recursion', False):
        raise RecursionError from rosException
      init_node(self.__node_name__, anonymous=True)
      return self.publish(command, _recursion=True)

  def activate(self, duration: float = None) -> None:
    """Activate the commander."""
    if isinstance(duration, int):
      return self.activate(float(duration))
    if isinstance(duration, complex):
      if duration.imag ** 2 > 1e-06:
        e = typeMsg('duration', duration, float)
        raise TypeError(e)
      return self.activate(duration.real)
    if not isinstance(duration, float):
      e = typeMsg('duration', duration, float)
      raise TypeError(e)
    if duration < 0:
      e = """Duration must be non-negative."""
      raise ValueError(e)
    if duration > 5000:
      e = """Duration must be less than 1000 ms."""
      raise ValueError(e)
    duration = Duration.from_sec(duration / 1000)
    command = AuxCommand(True, Time.now(), duration)
    self.publish(command)
