"""SubField implements descriptor protocol for ROS subscriber instances."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Never

from icecream import ic
from rospy import Subscriber
from vistutils.text import stringList, monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType

ic.configureOutput(includeContext=True)


class SubField:
  """SubField implements descriptor protocol for ROS subscriber instances."""

  __field_name__ = None
  __field_owner__ = None
  __topic_name__ = None
  __callback_name__ = None
  __callback_function__ = None

  def __init__(self, topicName: str, *args, **kwargs):
    """Initializes the descriptor"""
    self.__topic_name__ = topicName

  def getTopicName(self, ) -> str:
    """Getter-function for the topic name"""
    return self.__topic_name__

  def getTopicType(self, ) -> type:
    """Getter-function for the topic type"""
    return resolveTopicType(self.__topic_name__)

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner"""
    ic(owner, name)
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateName(self) -> str:
    """Getter-function for the private name"""
    return '_%s' % self.__field_name__

  def _getCallbackName(self) -> str:
    """Getter-function for the name at which to find the callback."""
    if self.__callback_name__ is None:
      e = """No callback function has been set for topic: '%s'!"""
      topicName = self.__topic_name__
      raise AttributeError(monoSpace(e % topicName))
    if isinstance(self.__callback_name__, str):
      return self.__callback_name__
    e = typeMsg('callback_name', self.__callback_name__, str)
    raise TypeError(e)

  def _setCallbackName(self, name: str) -> None:
    """Setter-function for the callback name"""
    if isinstance(name, str):
      self.__callback_name__ = name
    else:
      e = typeMsg('name', name, str)
      raise TypeError(e)

  def _getCallback(self, ) -> Callable:
    """Getter-function for the callback"""
    return self.__callback_function__

  def _setCallback(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for the callback"""
    if self.__callback_function__ is not None:
      e = """Callback function already set for topic: '%s'!"""
      topicName = self.__topic_name__
      raise AttributeError(monoSpace(e % topicName))
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__callback_function__ = callMeMaybe
    return callMeMaybe

  def _createSubscriber(self, instance: object, owner: type) -> None:
    """Creates the subscriber"""
    if instance is None:
      e = """Cannot instantiate subscriber on owning class, only on 
      instances!"""
      raise TypeError(monoSpace(e))
    callback = self._getCallback()
    cbName = callback.__name__
    instanceCallback = getattr(instance, cbName, None)
    if instanceCallback is None:
      e = """Callback function missing from instance!"""
      raise AttributeError(monoSpace(e))
    topicName = self.getTopicName()
    topicType = self.getTopicType()
    pvtName = self._getPrivateName()

    def getSub() -> Subscriber:
      """Getter-function for the subscriber"""
      return Subscriber(topicName, topicType, instanceCallback)

    setattr(instance, pvtName, getSub)
    self.__active_subscriber__ = getSub

  def run(self) -> None:
    """Runs the descriptor"""
    self.__active_subscriber__()

  def __get__(self, instance: object, owner: type, **kwargs) -> object:
    """Returns the subscriber"""
    ic(instance, owner, self.getTopicName())
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if getattr(instance, pvtName, None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createSubscriber(instance, owner)
      return self.__get__(instance, owner, _recursion=True)
    subscriber = getattr(instance, pvtName)
    return self
    if isinstance(subscriber, Subscriber):
      return subscriber
    e = typeMsg('subscriber', subscriber, Subscriber)
    raise TypeError(e)

  def __set__(self, *_) -> Never:
    """Illegal setter function"""
    e = """Subscribers are read only!"""
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal delete function"""
    e = """Subscribers are read only!"""
    raise TypeError(e)

  def CALL(self, callMeMaybe: Callable) -> Callable:
    """Decorator alias for callback setter function"""
    ic(callMeMaybe)
    return self._setCallback(callMeMaybe)
