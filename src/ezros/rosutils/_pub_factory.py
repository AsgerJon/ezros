"""ConstPublisher provides a Publisher remaining active."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import QTimer, QObject
from rospy import Publisher

from ezros.gui.shortnames import Precise
from ._topic_factory import TopicFactory


class PubFactory(TopicFactory):
  """ConstPublisher provides a Publisher remaining active."""

  __callback_factory__ = None

  def _createTimer(self, ) -> QTimer:
    """Creates an instance of QTimer"""
    timer = QTimer()
    timer.setInterval(int(round(1000 / self.__pub_rate__)))
    timer.setSingleShot(False)
    timer.setTimerType(Precise)
    return timer

  def _createPublisher(self, ) -> Publisher:
    """Creates an instance of Publisher"""
    return Publisher(self.__topic_name__,
                     self.__topic_type__,
                     queue_size=self.__queue_size__)

  def _getPublisherName(self) -> str:
    """Getter-function for the publisher name"""
    return '__%s_publisher__' % self.__field_name__

  def _getTimerName(self) -> str:
    """Getter-function for the timer name"""
    return '__%s_timer__' % self.__field_name__

  def _getCallbackName(self) -> str:
    """Getter-function for the callback name"""
    return '__%s_callMeMaybe__' % self.__field_name__

  def _generateCallback(self, instance: QObject) -> Callable:
    """Generates the callback function"""

    callback = self.__callback_factory__(instance)
    pubName = self._getPublisherName()

    def wrappedCallback(this) -> None:
      """The callback function"""
      pub = getattr(this, pubName, None)
      if pub is None:
        e = """Publisher not found!"""
        raise AttributeError(e)
      pub.publish(callback())

    return wrappedCallback

  def _instantiate(self, instance: QObject, owner: type) -> None:
    """Instantiates the Publisher"""
    if instance is None:
      e = """Cannot instantiate publisher on owning class, only on 
      instances!"""
      raise TypeError(e)
    callback = self._generateCallback(instance)
    setattr(instance, self._getPublisherName(), self._createPublisher())
    setattr(instance, self._getCallbackName(), callback)
    setattr(instance, self._getTimerName(), self._createTimer())

  def CALL(self, callMeMaybe: Callable) -> Callable:
    """Decorator for the callback function
    class LOL:
      lmao = PubField('topic', String, size=10)

      @lmao.CALL
      def _factory(self, ) -> Callable:

        def callback(msg: Any=None) -> Any:
          return String.fromStr('Hello, World!')

        return callback
    """
    self.__callback_factory__ = callMeMaybe
    return callMeMaybe
