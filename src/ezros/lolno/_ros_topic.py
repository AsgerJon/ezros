"""RosTopic provides a descriptor class for ROS topics. Roughly speaking,
the __get__ connects to a callback and __set__ to a publisher."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from PySide6.QtCore import QObject
from ezside.core import parseParent
from rospy import Subscriber
from vistutils.text import stringList, monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType


class RosTopic(QObject):
  """The RosTopic class provides a descriptor class for ROS topics. Roughly
  speaking, the __get__ connects to a callback and __set__ to a publisher."""

  __field_name__ = None
  __field_owner__ = None
  __topic_name__ = None
  __topic_type__ = None

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent()
    QObject.__init__(self, parent)
    nameKeys = stringList("""name, topic, topicName""")
    queueSizeKeys = stringList("""queueSize, queue_size""")
    values = dict(name=None, queueSize=None)
    names = ['name', 'queueSize']
    types = [str, int]
    defVals = [None, 10]
    Keys = [nameKeys, queueSizeKeys]
    for (name, type_, val0, keys) in zip(names, types, defVals, Keys):
      for key in keys:
        if key in kwargs:
          val = kwargs.get(key)
          if isinstance(val, type_):
            values[name] = val
            break
          e = typeMsg(key, val, type_)
          raise TypeError(e)
      else:
        for arg in args:
          if isinstance(arg, type_):
            values[name] = arg
            break
        else:
          values[name] = val0

  def getTopicName(self) -> str:
    """Returns the name of the topic."""
    return self.__topic_name__

  def getTopicType(self) -> type:
    """Returns the type of the topic."""
    return self.__topic_type__

  def __set_name__(self, owner: type, name: str) -> None:
    """Set the name of the field and owner."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def getFieldName(self) -> str:
    """Returns the name of the field."""
    return self.__field_name__

  def getPrivateFieldName(self) -> str:
    """Returns the private name of the field."""
    return '_%s' % self.__field_name__

  def _callbackFactory(self, ) -> Callable:
    """Returns the callback factory."""

    topicType = self.getTopicType()

    def callback(msg: topicType) -> None:
      """Returns the message as it is. """
      return msg

    return callback

  def subscriberFactory(self, ) -> None:
    """The subscriber factory."""
    subscriber = Subscriber(self.getTopicName(), self.getTopicType(),
                            self._callbackFactory())

  def autoPublish(self, value: Any) -> None:
    """Publish the value."""

  def __get__(self, instance: Any, owner: type) -> Any:
    """Connect to the callback."""
    if instance is None:
      return self
    return self.subscriberFactory()
