"""RosThread instances represent a named topic and PySide6 signal. The
class provides an internal subscriber and callback function,
which implements a signal emission containing the message through the Qt
signal protocol. Thus, this class provides a bridge between the
publisher/subscriber system of ROS with the event-driven system of Qt."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QThread, Signal
from icecream import ic
from rospy import Subscriber, spin

from ezros.rosutils import resolveTopicType, initNodeMaybe

ic.configureOutput(includeContext=True)


class RosThread(QThread):
  """RosThread instances represent a named topic and PySide6 signal. The
  class provides an internal subscriber and callback function,
  which implements a signal emission containing the message through the Qt
  signal protocol. Thus, this class provides a bridge between the
  publisher/subscriber system of ROS with the event-driven system of Qt."""

  __zero_time__ = None
  __topic_name__ = None
  __topic_type__ = None
  __ros_subscriber__ = None

  data = Signal(complex)

  def __init__(self, topicName: str, ) -> None:
    """Initializes the RosThread instance."""
    QThread.__init__(self)
    self.__zero_time__ = time.time()
    self.__topic_name__ = topicName
    self.__topic_type__ = resolveTopicType(self.__topic_name__)

  def run(self) -> None:
    """Runs the thread."""
    initNodeMaybe()
    self.__ros_subscriber__ = Subscriber(self.__topic_name__,
                                         self.__topic_type__,
                                         self.callback)
    spin()

  def callback(self, data: Any) -> None:
    """Callback function for the subscriber."""
    value = data.data
    when = data.header.stamp.to_sec()
    self.data.emit(when + value * 1j)
