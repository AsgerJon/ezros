"""This class combines QObject and ROS functionality on a topic level. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from rospy import Subscriber
import rostopic
from vistutils.fields import EmptyField

ic.configureOutput(includeContext=True)


class _Subscriber(Subscriber):
  """subclass implementing a hook in unregister"""

  def unregister(self) -> None:
    """Unregisters the subscriber. """
    Subscriber.unregister(self, )
    self.callback = None
    ic('Subscriber unregistered!')


class RosTopic:
  """This class combines QObject and ROS functionality on a topic level. """

  __topic_name__ = None
  __topic_type__ = None

  topicName = EmptyField()
  topicType = EmptyField()

  def __init__(self, topicName: str) -> None:
    """Initializes the RosTopic instance. """
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
