"""PubField provides a descriptor class wrapping the ROS publishers. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from rospy import Publisher
from vistutils.parse import maybe
from vistutils.text import monoSpace, stringList
from vistutils.waitaminute import typeMsg
from PySide6.QtCore import QObject

from ezros.rosutils import resolveTopicType

ShibokenType = type(QObject)


class PubField:
  """A descriptor class wrapping the ROS publishers. """

  __field_name__ = None
  __field_owner__ = None
  __topic_name__ = None
  __topic_type__ = None
  __queue_size__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the PubField"""
    self.__topic_name__ = args[0]
    self.__topic_type__ = args[1]
    self.__queue_size__ = kwargs.get('size', 10)

  def getTopicName(self, ) -> str:
    """Getter-function for the topic name"""
    return self.__topic_name__

  def getTopicType(self, ) -> type:
    """Getter-function for the topic type"""
    return self.__topic_type__

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner"""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateName(self) -> str:
    """Getter-function for the private name"""
    return '_%s' % self.__field_name__

  def _createPublisher(self, instance: object, owner: type) -> None:
    """Creates the publisher"""
    if instance is None:
      e = """Cannot instantiate publisher on owning class, only on 
      instances!"""
      raise TypeError(monoSpace(e))
    topicName = self.getTopicName()
    topicType = self.__topic_type__
    pvtName = self._getPrivateName()
    publisher = Publisher(topicName, topicType, queue_size=10)
    setattr(instance, pvtName, publisher)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Returns the publisher"""
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if getattr(instance, pvtName, None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPublisher(instance, owner)
      return self.__get__(instance, owner, _recursion=True)
    publisher = getattr(instance, pvtName)
    if isinstance(publisher, Publisher):
      return publisher
    e = typeMsg('publisher', publisher, Publisher)
    raise TypeError(e)

  def __set__(self, *_) -> Never:
    """Illegal setter function"""
    e = """Publishers are read only!"""
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal delete function"""
    e = """Publishers are read only!"""
    raise TypeError(e)
