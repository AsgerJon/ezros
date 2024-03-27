"""Talker wraps the Publisher. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Callable

from PySide6.QtCore import Signal, Slot
from attribox import AttriBox, this
from ezside.core import parseParent, Precise
from ezside.widgets import BaseWidget, Timer
from rospy import Publisher, Rate
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.rosutils import Pub


class Talker(BaseWidget):
  """Talker wraps the Publisher. """

  __topic_name__ = None
  __queue_size__ = None
  __filter_function__ = None

  qTalk = Signal(float)

  publisher = AttriBox[Pub](this)
  rate = AttriBox[Rate](this)
  timer = AttriBox[Timer](Precise, 20, singleShot=False)

  def __init__(self, topicName: str, *args) -> None:
    parent = parseParent()
    BaseWidget.__init__(self, parent)
    self.__topic_name__ = topicName
    self.__queue_size__ = None

  def getFilterFunction(self) -> Callable:
    """Getter-function for the filter function. This function is
    responsible for assigning a value to the current epoch time. If not
    provided the epoch time is returned. """
    if self.__filter_function__ is None:
      return lambda t: t
    if callable(self.__filter_function__):
      return self.__filter_function__
    e = typeMsg('filterFunction', self.__filter_function__, Callable)
    raise TypeError(e)

  def setFilterFunction(self, filterFunction: Callable) -> None:
    """Setter-function for the filter function."""
    if self.__filter_function__ is not None:
      e = """The filter function has already been set. """
      raise AttributeError(e)
    if callable(filterFunction):
      self.__filter_function__ = filterFunction
    else:
      e = typeMsg('filterFunction', filterFunction, Callable)
      raise TypeError(e)

  @Slot()
  def talk(self, ) -> None:
    """Publish a message to the topic."""
    rightNow = time.time()
    filterFunc = self.getFilterFunction()
    value = filterFunc(rightNow)
    self.publisher.publish(value)
    self.qTalk.emit(value)

  def connectActions(self) -> None:
    """Connect the actions."""
    self.timer.timeout.connect(self.talk)

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.timer.start()
