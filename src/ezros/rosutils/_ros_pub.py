"""RosPub wraps the rospy.Publisher class and simplifies the constructor
so that it requires only a representation of the topic. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezside.core import parseParent
from ezside.widgets import BaseWidget
from rospy import Publisher
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType


class Pub(BaseWidget):
  """RosPub wraps the rospy.Publisher class and simplifies the constructor
  so that it requires only a representation of the topic. """

  __fallback_queue_size__ = 10
  __topic_name__ = None
  __topic_type__ = None
  __queue_size__ = None

  def __init__(self, topicName: str, topicType: type, *args) -> None:
    """Initialize the publisher."""
    parent = parseParent()
    BaseWidget.__init__(self, parent)
    self.__topic_name__ = topicName
    self.__topic_type__ = topicType
    self.__queue_size__ = None
    for arg in args:
      if isinstance(arg, int):
        self.__queue_size__ = arg
        break
    else:
      self.__queue_size__ = self.__fallback_queue_size__

    BaseWidget.__init__(self,
                        topicName,
                        topicType,
                        queue_size=self.__queue_size__)
