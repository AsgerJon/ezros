"""RosThread instances represent a named topic and PySide6 signal. The
class provides an internal subscriber and callback function,
which implements a signal emission containing the message through the Qt
signal protocol. Thus, this class provides a bridge between the
publisher/subscriber system of ROS with the event-driven system of Qt."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Callable

from PySide6.QtCore import QThread, Signal, Slot
from icecream import ic
from msgs.msg import Float32Stamped
from rospy import Subscriber, spin, ROSInitException
from rospy.core import is_initialized, is_shutdown
from rospy.rostime import wallsleep
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType, initNodeMaybe, RosThread

ic.configureOutput(includeContext=True)


class SubRos(RosThread):
  """RosThread instances represent a named topic and PySide6 signal. The
  class provides an internal subscriber and callback function,
  which implements a signal emission containing the message through the Qt
  signal protocol. Thus, this class provides a bridge between the
  publisher/subscriber system of ROS with the event-driven system of Qt."""

  __ros_subscriber__ = None

  data = Signal(object)

  def run(self) -> None:
    """First, the method creates a ROS subscriber instance at the given
    topic name and type with the default callback which emits the data
    signal. Then, the method enters a loop where it sleeps for 0.5 seconds
    and checks if the thread should continue running. If not, the subscriber
    is unregistered and the method exits."""
    name, type_, = self.getTopicName(), self.getTopicType()
    self.__ros_subscriber__ = Subscriber(name, type_, self._callback)
    while self.__allow_run__ and not is_shutdown():
      wallsleep(0.5)
    self.__ros_subscriber__.unregister()

  def _callback(self, data: Float32Stamped) -> None:
    """Callback function for the subscriber. Subclasses may reimplement
    this method if more complex behavior is needed. The default behavior
    is to emit the data signal with the received message."""
    self.setValue(data)
    self.data.emit(self.getValue())
