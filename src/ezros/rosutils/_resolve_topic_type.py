"""The getTopicType function receives the name of a topic and returns the
expected message type. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

from roslib.message import get_message_class
from rospy import get_published_topics
from vistutils.text import monoSpace


def resolveTopicType(topicName: str) -> Optional[type]:
  """Get the expected message type of given topic."""
  if topicName is None:
    return None
  topicNames = [name.lower() for name in topicName.split('/') if name]
  topicType = None
  topics = get_published_topics()
  for topic, type_ in topics:
    names = [name.lower() for name in topic.split('/') if name]
    if len(names) != len(topicNames):
      continue
    elif all([a == b for (a, b) in zip(names, topicNames)]):
      return get_message_class(type_)
  else:
    e = """Unable to recognize name: '%s' as a proper name for a published 
    topic!"""
    raise ValueError(monoSpace(e % topicName))
