"""Factory for creating publishers."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import Publisher

from ezros.rosutils import getTopicType, validateInitialized


def publisherFactory(topicName: str, nodeName: str = None) -> Publisher:
  """Create a publisher for a given topic and message type."""
  nodeName = 'Test' if nodeName is None else nodeName
  validateInitialized(nodeName)
  messageType = getTopicType(topicName)
  return Publisher(topicName, messageType, queue_size=10)
