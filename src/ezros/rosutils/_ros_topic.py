"""This class combines QObject and ROS functionality on a topic level. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from random import randint
from typing import Callable, TYPE_CHECKING

from PySide6.QtCore import QObject
from icecream import ic
from rospy import Subscriber, init_node
from rospy.core import is_initialized
import rostopic
from vistutils.fields import EmptyField
from vistutils.parse import maybe

ic.configureOutput(includeContext=True)

SubFactory = Callable[[Callable], Subscriber]


class RosTopic(QObject):
  """This class combines QObject and ROS functionality on a topic level. """

  __topic_name__ = None
  __topic_type__ = None

  topicName = EmptyField()
  topicType = EmptyField()

  def __init__(self, topicName: str) -> None:
    """Initializes the RosTopic instance. """
    QObject.__init__(self)
    self.__topic_name__ = topicName
    topicType = [*rostopic.get_topic_class(topicName), ]
    while topicType:
      item = topicType.pop(0)
      if isinstance(item, type):
        self.__topic_type__ = item
        break
    else:
      e = """Unable to resolve name: '%s' as the named of a topic!"""
      raise ValueError(e)

  @topicName.GET
  def _getTopicName(self) -> str:
    """Getter-function for the topic name. """
    return self.__topic_name__

  @topicType.GET
  def _getTopicType(self) -> type:
    """Getter-function for the topic type. """
    return self.__topic_type__

  def __str__(self) -> str:
    """Returns the string representation of the RosTopic. """
    clsName = self.__class__.__name__
    rosName = self.__topic_name__
    rosType = self.__topic_type__.__name__
    return '%s[%s]' % (rosName, rosType)

  def __repr__(self) -> str:
    """Returns the string representation of the RosTopic. """
    clsName = self.__class__.__name__
    rosName = self.__topic_name__
    rosType = self.__topic_type__.__name__
    return '%s(%s)' % (clsName, rosName)

  def subCreatorFactory(self, nodeName: str = None, **kwargs) -> SubFactory:
    """Returns a decorator creating a subscriber with the decorated
    callable as the callback."""

    anonFlag = kwargs.get('anonymous', False)
    nodeName = kwargs.get('nodeName', nodeName)
    if nodeName is None:
      anonFlag = True
      nodeName = 'Node_%d' % randint(0, 2 ** 32)

    def subCreator(callMeMaybe: Callable, ) -> Subscriber:
      """Creates a subscriber with the callback as the callback."""
      if not is_initialized():
        init_node(nodeName, anonymous=anonFlag)
      rosName, rosType = self.topicName, self.topicType
      if TYPE_CHECKING:
        assert isinstance(rosName, str) and isinstance(rosType, type)
      return Subscriber(rosName, rosType, callMeMaybe)

    return subCreator
