#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen

"""TopicFactory provides functionality shared between factories creating
Publishers and Subscribers."""
from __future__ import annotations

from PySide6.QtCore import QObject
from roslib.message import get_message_class
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg


class TopicFactory:
  """TopicFactory provides functionality shared between factories creating
  Publishers and Subscribers."""

  __field_name__ = None
  __field_owner__ = None
  __topic_name__ = None
  __topic_type__ = None
  __pub_rate__ = None
  __queue_size__ = None

  @staticmethod
  def _parseKwargs(**kwargs) -> dict:
    """Parse the keyword arguments"""
    nameKeys = stringList("""name, topicName, topic_name""")
    typeKeys = stringList("""type, topicType, topic_type""")
    rateKeys = stringList("""rate, pubRate, pub_rate""")
    sizeKeys = stringList("""size, queueSize, queue_size""")
    Keys = [nameKeys, typeKeys, rateKeys, sizeKeys]
    names = stringList("""name, type, rate, size""")
    types = [str, type, float, int]
    usedKeys = []
    parsed = {name: None for name in names}
    for (keys, name, type_) in zip(Keys, names, types):
      for key in keys:
        if key in kwargs and key not in usedKeys:
          val = kwargs[key]
          if isinstance(val, type_):
            parsed[name] = kwargs[key]
            usedKeys.append(key)
            break
      else:
        parsed[name] = None
    return parsed

  @staticmethod
  def _parseArgs(*args) -> dict:
    """Parses positional arguments"""
    names = stringList("""name, type, rate, size""")
    types = [str, type, float, int]
    parsed = {name: None for name in names}
    if not args:
      return {name: None for name in names}
    usedArgs = []
    for (name, type_,) in zip(names, types):
      for arg in args:
        if isinstance(arg, type_) and parsed[name] is None:
          if arg not in usedArgs:
            parsed[name] = arg
            usedArgs.append(arg)
    return parsed

  def _parse(self, *args, **kwargs) -> None:
    """Parse the arguments"""
    parsedKwargs = self._parseKwargs(**kwargs)
    parsedArgs = self._parseArgs(*args)
    parsed = {**parsedKwargs, **parsedArgs}
    topicName = parsed.get('name', None)
    if topicName is None:
      raise NameError('Required topicName not found!')
    if not isinstance(topicName, str):
      e = typeMsg('topicName', topicName, str)
      raise TypeError(e)
    self.__topic_name__ = topicName
    topicType = parsed.get('type', None)
    if topicType is None:
      topicType = get_message_class(topicName)
    if not isinstance(topicType, type):
      e = typeMsg('topicType', topicType, type)
      raise TypeError(e)
    self.__topic_type__ = topicType
    pubRate = parsed.get('rate', 10)
    if not isinstance(pubRate, (float, int)):
      e = typeMsg('pubRate', pubRate, float)
      raise TypeError(e)
    self.__pub_rate__ = pubRate
    queueSize = parsed.get('size', 10)
    if not isinstance(queueSize, int):
      e = typeMsg('queueSize', queueSize, int)
      raise TypeError(e)
    self.__queue_size__ = queueSize

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the descriptor"""
    self._parse(*args, **kwargs)

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner"""
    if not isinstance(owner, type(QObject)):
      e = typeMsg('owner', owner, type(QObject))
      raise TypeError(e)
    self.__field_name__ = name
    self.__field_owner__ = owner

  def getFieldName(self) -> str:
    """Getter-function for the field name"""
    return self.__field_name__

  def getFieldOwner(self) -> type:
    """Getter-function for the field owner"""
    return self.__field_owner__

  def getTopicName(self) -> str:
    """Getter-function for the topic name"""
    return self.__topic_name__

  def getTopicType(self) -> type:
    """Getter-function for the topic type"""
    return self.__topic_type__

  def getPubRate(self) -> float:
    """Getter-function for the publishing rate"""
    return self.__pub_rate__

  def getQueueSize(self) -> int:
    """Getter-function for the queue size"""
    return self.__queue_size__

  def _getPrivateName(self) -> str:
    """Getter-function for the private name"""
    return '_%s' % self.__field_name__
