"""PubRos provides a QThread-based ROS publisher. Each instance holds an
internal state that is periodically published on the given topic name. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from math import floor
from typing import Any, TYPE_CHECKING

from rospy import Publisher, is_shutdown
from rospy.rostime import wallsleep
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.rosutils import RosThread

from genpy import Duration, Time, Message


class PubRos(RosThread):
  """PubRos provides a QThread-based ROS publisher. Each instance holds an
  internal state that is periodically published on the given topic name. """

  __ros_publisher__ = None
  __publish_duration__ = None
  __fallback_duration__ = 0.5

  def getDuration(self, **kwargs) -> Duration:
    """Getter-function for the 'publish' period in seconds. """
    if self.__publish_duration__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.setDuration(self.__fallback_duration__)
      return self.getDuration(_recursion=True)
    period = maybe(self.__publish_duration__, self.__fallback_duration__)
    seconds = floor(period)
    nanoSeconds = (period - seconds) * 1e09
    return Duration(int(seconds), int(nanoSeconds))

  def setDuration(self, duration: Any) -> None:
    """Setter-function for the 'publish' period in seconds."""
    if isinstance(duration, Duration):
      self.__publish_duration__ = float(duration.to_sec())
    elif isinstance(duration, int):
      self.__publish_duration__ = float(duration)
    elif isinstance(duration, float):
      self.__publish_duration__ = duration
    if self.__publish_duration__ <= 0:
      e = """The period must be a positive number!"""
      raise ValueError(e)

  def run(self) -> None:
    """First, the method creates a ROS publisher instance at the given
    topic name and type. Then, the method enters a loot where it
    periodically transmits its internal state. """
    name, type_ = self.getTopicName(), self.getTopicType()
    self.__ros_publisher__ = Publisher(name, type_, queue_size=10)
    while self.__allow_run__ and not is_shutdown():
      self.__ros_publisher__.publish(self.getValue())
      wallsleep(self.getDuration().to_sec())
