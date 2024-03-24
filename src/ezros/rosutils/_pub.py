"""Pub wraps the Publisher class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from genpy import Message
from numpy.fft import irfft
from numpy.random import normal
import numpy as np
from typing import Tuple, Any

from rospy import Publisher
from rospy.impl.transport import Transport
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType


class Pub(Publisher):
  """WrappedPublisher wraps the Publisher class to provide additional
  functionalities"""

  __fallback_queue_size__ = 10
  __topic_name__ = None
  __topic_type__ = None
  __queue_size__ = None

  def __init__(self, instance: Any) -> None:
    name = getattr(instance, '__topic_name__', None)
    if name is None:
      e = """The first argument of the Publisher class must be the name of 
      the
      topic. """
      raise AttributeError(e)
    self.setTopicName(name)
    type_ = resolveTopicType(name)
    self.setTopicType(type_)
    queueSize = getattr(instance, '__queue_size__', None)
    self.setQueueSize(maybe(queueSize, self.__fallback_queue_size__))
    Publisher.__init__(self, name, type_, queue_size=self.getQueueSize())

  def getTopicName(self) -> str:
    """Get the name of the topic."""
    if isinstance(self.__topic_name__, str):
      return self.__topic_name__
    if self.__topic_name__ is None:
      e = """The topic name is not set. """
      raise AttributeError(e)
    e = typeMsg('topicName', self.__topic_name__, str)
    raise TypeError(e)

  def setTopicName(self, topicName: str) -> None:
    """Setter-function for the topic name."""
    if isinstance(topicName, str):
      self.__topic_name__ = topicName
    else:
      e = typeMsg('topicName', topicName, str)
      raise TypeError(e)

  def getTopicType(self) -> type:
    """Get the type of the topic."""
    if isinstance(self.__topic_type__, type):
      return self.__topic_type__
    if self.__topic_type__ is None:
      e = """The topic type is not set. """
      raise AttributeError(e)
    e = typeMsg('topicType', self.__topic_type__, type)
    raise TypeError(e)

  def setTopicType(self, topicType: type) -> None:
    """Setter-function for the topic type."""
    if isinstance(topicType, type):
      self.__topic_type__ = topicType
    else:
      e = typeMsg('topicType', topicType, type)
      raise TypeError(e)

  def getQueueSize(self) -> int:
    """Get the queue size."""
    queueSize = maybe(self.__queue_size__, self.__fallback_queue_size__)
    if isinstance(queueSize, int):
      return queueSize
    e = typeMsg('queueSize', queueSize, int)
    raise TypeError(e)

  def setQueueSize(self, queueSize: int) -> None:
    """Setter-function for the queue size."""
    if isinstance(queueSize, int):
      self.__queue_size__ = queueSize
    else:
      e = typeMsg('queueSize', queueSize, int)
      raise TypeError(e)

  def instantiateMessage(self, value: float = None) -> Any:
    """Instantiate a new message object."""
    cls = self.getTopicType()
    item = cls()
    if not isinstance(value, (int, float)):
      e = typeMsg('value', value, float)
      raise TypeError(e)
    item.data = item.data if value is None else value
    if value is None:
      return item
    item.data = value
    return item

  def publish(self, *args) -> None:
    """Publish a message to the topic."""
    connection, value, message = None, None, None
    for arg in args:
      if isinstance(arg, Transport) and connection is None:
        connection = arg
      elif isinstance(arg, Message) and message is None:
        message = arg
      elif isinstance(arg, (int, float)) and value is None:
        value = arg
    if message is None:
      if value is None:
        e = """Unable to parse message object!"""
        raise ValueError(e)
      if not isinstance(value, (int, float)):
        e = typeMsg('value', value, float)
        raise TypeError(e)
      message = self.instantiateMessage(value)
    if not isinstance(message, Message):
      e = typeMsg('message', message, Message)
      raise TypeError(e)
    if connection is None:
      return Publisher.publish(self, message)
    if isinstance(connection, Transport):
      return Publisher.publish(self, message, connection)
    e = typeMsg('connection', connection, Transport)
    raise TypeError(e)
