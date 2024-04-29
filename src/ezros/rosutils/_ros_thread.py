"""RosThread provides an abstract baseclass for ROS publishers and
subscribers that require dedicated threads. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QThread, Slot
from genpy import Message
from rospy import ROSInitException
from rospy.core import is_initialized
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
import std_msgs.msg


class RosThread(QThread):
  """RosThread provides an abstract baseclass for ROS publishers and
  subscribers that require dedicated threads. """

  __allow_run__ = None
  __internal_value__ = None
  __topic_name__ = None
  __topic_type__ = None

  def __init__(self, topicName: str, topicType: type = None) -> None:
    """Initializes the RosThread instance."""
    QThread.__init__(self, )
    self.__topic_name__ = topicName
    if topicType is None:
      topicType = resolveTopicType(topicName)
    if not issubclass(topicType, Message):
      e = typeMsg('topicType', topicType, Message)
      raise TypeError(e)
    self.__topic_type__ = topicType
    self.__allow_run__ = False
    self.__internal_value__ = None

  def start(self, *args, **kwargs) -> None:
    """Reimplementation of the start method which ensures that the ROS
    node is properly initialized before starting the thread. The method
    then sets the allow_run flag to True and starts the thread by invoking
    the parent method. """
    if not is_initialized():
      e = """ROS does not appear to be properly initialized!"""
      raise ROSInitException(e)
    self.__allow_run__ = True
    QThread.start(self, *args, **kwargs)

  @Slot()
  def stop(self, ) -> None:
    """This method stops the thread by setting the allow_run flag to
    False."""
    self.__allow_run__ = False

  def getTopicName(self) -> str:
    """Getter-function for topic name"""
    if self.__topic_name__ is None:
      e = """The topic name has not been set!"""
      raise AttributeError(e)
    if isinstance(self.__topic_name__, str):
      return self.__topic_name__
    e = typeMsg('__topic_name__', self.__topic_name__, str)
    raise TypeError(e)

  def getTopicType(self) -> type:
    """Getter-function for topic type"""
    if self.__topic_type__ is None:
      e = """The topic type has not been set!"""
      raise AttributeError(e)
    if issubclass(self.__topic_type__, Message):
      return self.__topic_type__
    e = typeMsg('__topic_type__', self.__topic_type__, type)
    raise TypeError(e)

  def getValue(self, ) -> Message:
    """Getter-function for the internal value."""
    return self.__internal_value__

  def setValue(self, value: Message) -> None:
    """Setter-function for the internal value."""
    if isinstance(value, self.getTopicType()):
      self.__internal_value__ = value
    else:
      e = typeMsg('value', value, self.getTopicType())
      raise TypeError(e)
