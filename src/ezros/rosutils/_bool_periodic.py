"""BoolPeriodic provides a subclass of BoolPub that turns a signal on and off
according to a specified program. This program is defined by an on period
and an off period that repeats."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, TYPE_CHECKING

from PySide6.QtCore import Slot, QObject
from attribox import AttriBox
from ezside.core import EZTimer, Precise
from genpy import Duration, Message
from rospy import Time, Publisher
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.rosutils import BoolPub

if TYPE_CHECKING:
  auxDict = {
    '__slots__': ['activate', 'when', 'for_duration'],
    '_slot_types_': ['bool', 'time', 'duration']
  }
  AuxCommand = type('AuxCommand', (Message,), auxDict)
else:
  try:
    from msgs.msg import AuxCommand
  except ModuleNotFoundError as moduleNotFoundError:
    _e = """The AuxCommand message type is not available!"""
    raise ImportError(_e) from moduleNotFoundError


class BoolPeriodic(QObject):
  """BoolPeriodic provides a subclass of BoolPub that periodically turns
  a signal on or off."""

  __ros_publisher__ = None
  __low_epoch__ = None
  __high_epoch__ = None
  __low_fallback__ = 1
  __high_fallback__ = 1

  offTime = AttriBox[EZTimer](1000, True, Precise)
  onTime = AttriBox[EZTimer](1000, True, Precise)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the BoolPeriodic instance."""
    parent, topicName, topicType = None, None, None
    nameKeys = stringList("""name, topicName, topic""")
    typeKeys = stringList("""type, topicType, msgType""")
    parentKeys = stringList("""parent, parentWidget, main""")
    types = dict(topicName=str, topicType=type, parent=QObject)
    values = dict(topicName=None, topicType=None, parent=None)
    defVals = dict(topicName=None, topicType=AuxCommand, parent=None)
    KEYS = [nameKeys, typeKeys, parentKeys]
    for (keys, (name, type_)) in zip(KEYS, types.items()):
      for key in keys:
        if key in kwargs:
          val = kwargs[key]
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
          values[name] = defVals[name]
    if values['topicName'] is None:
      e = """Missing required argument: topicName!"""
      raise ValueError(e)
    self.__topic_name__ = values['topicName']
    if values['topicType'] != AuxCommand:
      e = """Unexpected topicType: '%s'!""" % str(values['topicType'])
      raise TypeError(e)
    self.__topic_type__ = values['topicType']
    if values['parent'] is None:
      QObject.__init__(self)
    elif isinstance(values['parent'], QObject):
      QObject.__init__(self, values['parent'])
    else:
      e = typeMsg('parent', values['parent'], QObject)
      raise TypeError(e)
    self.offTime.timeout.connect(self._pubHigh)
    self.offTime.timeout.connect(self.onTime.start)
    self.onTime.timeout.connect(self._pubLow)
    self.onTime.timeout.connect(self.offTime.start)

  @staticmethod
  def _validateTime(period: Any) -> float:
    """Validate the time."""
    out = None
    if isinstance(period, Duration) and out is None:
      out = period.to_sec()
    elif isinstance(period, int):
      out = float(period)
    elif isinstance(period, float):
      out = period
    if out is None:
      e = typeMsg('time', period, float)
      raise TypeError(e)
    if out > 0:
      return out
    e = """The period must be a positive number!"""
    raise ValueError(e)

  def setLowEpoch(self, period: Any) -> None:
    """Setter-function for the duration of low signal"""
    self.__low_epoch__ = self._validateTime(period)
    wasActive = self.offTime.isActive()
    if wasActive:
      self.offTime.stop()
    self.offTime.setInterval(self.__low_epoch__ * 1e03)
    if wasActive:
      self.offTime.start()

  def getLowEpoch(self, **kwargs) -> float:
    """Getter-function for the duration of low signal"""
    if self.__low_epoch__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.setLowEpoch(self.__low_fallback__)
      return self.getLowEpoch(_recursion=True)
    return self._validateTime(self.__low_epoch__)

  def setHighEpoch(self, period: Any) -> None:
    """Setter-function for the duration of high signal"""
    self.__high_epoch__ = self._validateTime(period)
    wasActive = self.onTime.isActive()
    if wasActive:
      self.onTime.stop()
    self.onTime.setInterval(self.__high_epoch__ * 1e03)
    if wasActive:
      self.onTime.start()

  def getHighEpoch(self, **kwargs) -> float:
    """Getter-function for the duration of high signal"""
    if self.__high_epoch__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.setHighEpoch(self.__high_fallback__)
      return self.getHighEpoch(_recursion=True)
    return self._validateTime(self.__high_epoch__)

  def getLowMessage(self) -> AuxCommand:
    """Return the low message."""
    duration = Duration.from_sec(self.getLowEpoch() * 1.5)
    return AuxCommand(False, Time(), duration)

  def getHighMessage(self) -> AuxCommand:
    """Return the high message."""
    duration = Duration.from_sec(self.getHighEpoch() * 1.5)
    return AuxCommand(True, Time(), duration)

  def _pubHigh(self) -> None:
    """Publishes high message"""
    self._getPublisher().publish(self.getHighMessage())

  def _pubLow(self) -> None:
    """Publishes low message"""
    self._getPublisher().publish(self.getLowMessage())

  def getTopicName(self) -> str:
    """Get the topic name."""
    return self.__topic_name__

  def getTopicType(self) -> type:
    """Get the topic type."""
    return self.__topic_type__

  def _getPublisher(self, **kwargs) -> Publisher:
    """Get the publisher."""
    if self.__ros_publisher__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      name, type_ = self.getTopicName(), self.getTopicType()
      self.__ros_publisher__ = Publisher(name, type_, queue_size=10)
      return self._getPublisher(_recursion=True)
    if isinstance(self.__ros_publisher__, Publisher):
      return self.__ros_publisher__
    e = typeMsg('self.__ros_publisher__', self.__ros_publisher__, Publisher)
    raise TypeError(e)

  def start(self) -> None:
    """Start the signal."""
    self.onTime.start()
    self.offTime.start()

  def stop(self) -> None:
    """Stop the signal."""
    self.onTime.stop()
    self.offTime.stop()
