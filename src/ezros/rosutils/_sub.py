"""Sub provides a wrapper for the Subscriber class. Please note that
unlike the Pub wrapper, Sub does not actually subclass Subscriber. Instead,
it owns a subscriber that it creates when requested. This allows deference
of the callback allowing callbacks to be set after instantiation for
example as a decorator. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtWidgets import QWidget
from ezside.widgets import BaseWidget
from icecream import ic
from rospy import Subscriber, init_node
from vistutils.text import monoSpace, joinWords
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
import vistutils.text

ic.configureOutput(includeContext=True)


class Sub:
  """Sub provides a wrapper for the Subscriber class. Please note that
  unlike the Pub wrapper, Sub does not actually subclass Subscriber. Instead,
  it owns a subscriber that it creates when requested. This allows deference
  of the callback allowing callbacks to be set after instantiation for
  example as a decorator. """

  __topic_name__ = None
  __topic_type__ = None
  __callback_function__ = None
  __subscriber_instance__ = None

  def __init__(self, *args) -> None:
    topicName, widget = None, None
    ic(args)
    ic(topicName)
    ic(args[0])
    for arg in args:
      if isinstance(arg, str) and topicName is None:
        topicName = arg
      elif isinstance(arg, QWidget) and widget is None:
        ic(arg)
        widget = arg
    if widget is not None:
      if isinstance(widget, QWidget):
        topicName = getattr(widget, '__topic_name__', None)
    if topicName is None:
      e = """The topic name is required. """
      raise AttributeError(e)
    self.__topic_name__ = topicName
    self.__topic_type__ = resolveTopicType(topicName)

  def setCallback(self, callMeMaybe: Callable) -> Callable:
    """Set the callback for the subscriber."""
    if self.__callback_function__ is None:
      if callable(callMeMaybe):
        self.__callback_function__ = callMeMaybe
        return callMeMaybe
      e = typeMsg('callback', callMeMaybe, Callable)
      raise TypeError(e)
    else:
      e = """The callback function has already been set. """
      raise AttributeError(e)

  def getCallback(self) -> Callable:
    """Get the callback for the subscriber."""
    if self.__callback_function__ is not None:
      if callable(self.__callback_function__):
        return self.__callback_function__
      e = typeMsg('callback', self.__callback_function__, Callable)
      raise TypeError(e)
    e = """The callback function has not been set. """
    raise AttributeError(e)

  def CALL(self, callMeMaybe: Callable) -> Callable:
    """Decorator for setting the callback."""
    return self.setCallback(callMeMaybe)

  def getSubscriber(self, **kwargs) -> Subscriber:
    """Getter-function for the Subscriber"""
    if self.__subscriber_instance__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createSubscriber()
      return self.getSubscriber(_recursion=True)
    if isinstance(self.__subscriber_instance__, Subscriber):
      return self.__subscriber_instance__
    e = typeMsg('subscriber', self.__subscriber_instance__, Subscriber)
    raise TypeError(e)

  def createSubscriber(self, ) -> None:
    """Create the subscriber."""
    init_node('Test', anonymous=True, )
    name, type_ = self.__topic_name__, self.__topic_type__
    callback = self.getCallback()
    if self.__subscriber_instance__ is not None:
      e = """The subscriber has already been created. """
      raise AttributeError(e)
    if any([i is None for i in [name, type_, callback]]):
      named = dict(name=name, type_=type_, callback=callback)
      missing = [k for k, v in named.items() if v is None]
      e = """Unable to create subscriber. Missing the following 
      attributes: %s""" % joinWords(*missing)
      raise AttributeError(monoSpace(e))
    if not callable(callback):
      e = typeMsg('callback', callback, Callable)
      raise TypeError(e)
    if not isinstance(name, str):
      e = typeMsg('name', name, str)
      raise TypeError(e)
    if not isinstance(type_, type):
      e = typeMsg('type_', type_, type)
      raise TypeError(e)
    sub = Subscriber(name, type_, callback, )
    setattr(self, '__subscriber_instance__', sub)

  def __str__(self, ) -> str:
    """String representation"""
    msg = """Instance of SubscriberWrapper for topic name: %s"""
    return msg % self.__topic_name__
