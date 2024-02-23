"""The validateTopicName function validates that a topic name does refer
to a published topic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from rospy import get_published_topics

from ezros.rosutils import validateInitialized

ic.configureOutput(includeContext=True)


def validateTopicName(topicName: str) -> str:
  """Validate that a topic name refers to a published topic."""
  validateInitialized()
  topics = get_published_topics()
  for topic, _ in topics:
    if topic in topicName or topicName in topic:
      return topicName
  else:
    raise NameError(topicName)
