"""Topic instances represent a named topic in ROS. They include SUB and
PUB decorators to decorate functions as subscribers and publishers
respectively."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import get_published_topics
from vistutils.text import monoSpace


class Topic:
  """Topic instances represent a named topic in ROS. They include SUB and
  PUB decorators to decorate functions as subscribers and publishers
  respectively."""

  @staticmethod
  def _validateTopicName(topicName: str) -> None:
    """Validates a topic name"""
    for (name, type_) in get_published_topics():
      names = name.split('/')
      if topicName.lower().replace('/', '') in names:
        return name
    e = """No topic with name: '%s' found!""" % topicName
    raise NameError(monoSpace(e))

  def __init__(self, topicName: str, ) -> None:
    self.topicName = topicName
