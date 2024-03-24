"""RosSub class wraps the rospy.Subscriber class and implements deferred
definition of the callback function allowing instances to act like
decorators. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from ezside.core import parseParent
from ezside.widgets import BaseWidget

from ezros.rosutils import Pub


class Sub(BaseWidget):
  """RosSub class wraps the rospy.Subscriber class and implements deferred
  definition of the callback function allowing instances to act like
  decorators. """

  __callback_function__ = None

  def __init__(self, topicName: str, topicType: type, *args) -> None:
    """Initialize the subscriber."""
    parent = parseParent()
    BaseWidget.__init__(self, parent)
    self.__topic_name__ = topicName
    self.__topic_type__ = topicType
    for arg in args:
      if callable(arg):
        self.setCallback(arg)
        break

  def setCallback(self, callMeMaybe: Callable) -> Callable:
    """Set the callback function."""
    self.setCallback(callMeMaybe)
    return callMeMaybe

  def CALL(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the callback function."""
    return self.setCallback(callMeMaybe)
