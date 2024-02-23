"""The getTopicType function receives the name of a topic and returns the
expected message type. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from roslib.message import get_message_class
from rospy import get_published_topics

from ezros.rosutils import validateTopicName


def getTopicType(topicName: str) -> type:
  """Get the expected message type of given topic."""
  topicName = validateTopicName(topicName)
  topicType = None
  topics = get_published_topics()
  for topic, type_ in topics:
    if topic == topicName:
      topicType = type_
      break
  else:
    raise NameError(topicName)
  return get_message_class(topicType)
