"""RosTopic wraps a named ros topic and resolves the type. The class
provides factories for creating publishers and subscribers. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from rospy import Publisher, Subscriber
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType


class RosTopic:
  """A class for wrapping a named ROS topic and resolving its type."""

  __fallback_queue_size__ = 10

  __topic_name__ = None
  __topic_type__ = None
  __queue_size__ = None

  def __init__(self, name: str, *args) -> None:
    """Create a new RosTopic object."""
    self.__topic_name__ = name
    self.__topic_type__ = resolveTopicType(name)

  def getQueueSize(self) -> int:
    """Get the queue size of the topic."""
    queueSize = maybe(self.__queue_size__, self.__fallback_queue_size__)
    if isinstance(queueSize, int):
      return queueSize
    e = typeMsg('queueSize', queueSize, int)
    raise TypeError(e)

  def setQueueSize(self, queueSize: int) -> None:
    """Set the queue size of the topic."""
    if isinstance(queueSize, int):
      self.__queue_size__ = queueSize
    else:
      e = typeMsg('queueSize', queueSize, int)
      raise TypeError(e)

  def _createPublisher(self, ) -> Publisher:
    """Create a new publisher for the topic."""
    name, type_ = self.__topic_name__, self.__topic_type__
    return Publisher(name, type_, queue_size=self.getQueueSize(), )

  def _createSubscriber(self, callMeMaybe: Callable) -> Subscriber:
    """Create a new subscriber for the topic."""
    name, type_ = self.__topic_name__, self.__topic_type__
    return Subscriber(name, type_, callMeMaybe, )
