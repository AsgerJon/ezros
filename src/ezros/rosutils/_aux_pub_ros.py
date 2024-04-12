"""AuxPubRos specifically publishes ROS messages of type AuxCommand."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QThread, Slot, QTimer, Signal
from msg import AuxCommand
from rospy import Publisher, Time
from vistutils.parse import maybe
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
from ezros.settings import Defaults
from ezros.utils import EmptyField


class AuxPubRos(QThread):
  """AuxPubRos is a QThread that publishes ROS messages of type
  AuxCommand."""
  __inner_state__ = None
  __inner_period__ = None
  __inner_timer__ = None
  __inner_publisher__ = None
  __fallback_period__ = Defaults.publisherInterval
  __topic_name__ = None

  period = EmptyField()
  timer = EmptyField()
  state = EmptyField()
  publisher = EmptyField()
  topicName = EmptyField()

  published = Signal(bool)
  stateChanged = Signal(bool)

  @period.GET
  def _getPeriod(self) -> int:
    """Getter-function for period"""
    period = maybe(self.__inner_period__, self.__fallback_period__)
    if isinstance(period, int):
      return period
    e = typeMsg('__inner_period__', self.__inner_period__, int)
    raise TypeError(e)

  def _createTimer(self, ) -> None:
    """Creator-function for timer"""
    if self.__inner_timer__ is not None:
      e = """Tried creating timer, but timer already exists!"""
      raise RuntimeError(e)
    self.__inner_timer__ = QTimer()
    if isinstance(self.period, int):
      self.__inner_timer__.setInterval(self.period)
    self.__inner_timer__.setSingleShot(False)
    self.__inner_timer__.timeout.connect(self._publish)

  @timer.GET
  def _getTimer(self, **kwargs) -> QTimer:
    """Getter-function for timer"""
    if self.__inner_timer__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createTimer()
      return self._getTimer(_recursion=True)
    if isinstance(self.__inner_timer__, QTimer):
      return self.__inner_timer__
    e = typeMsg('__inner_timer__', self.__inner_timer__, QTimer)
    raise TypeError(e)

  @state.GET
  def _getState(self) -> bool:
    """Getter-function for state"""
    return True if self.__inner_state__ else False

  @state.SET
  def _setState(self, value: bool) -> None:
    """Setter-function for state"""
    self.__inner_state__ = True if value else False

  def _getCommand(self) -> AuxCommand:
    """Getter-function for the ROS message."""
    auxCommand = AuxCommand()
    auxCommand.activate = self.__inner_state__
    auxCommand.when = Time.now()
    auxCommand.for_duration = self.__inner_period__ * 2
    return auxCommand

  def _createPublisher(self) -> None:
    """Creator-function for the ROS publisher."""
    if self.__inner_publisher__ is not None:
      e = """Tried creating publisher, but publisher already exists!"""
      raise RuntimeError(e)
    queue = Defaults.publisherQueue
    name = self.topicName
    if not isinstance(name, str):
      e = typeMsg('name', name, str)
      raise TypeError(e)
    self.__inner_publisher__ = Publisher(name, AuxCommand, queue_size=queue)

  @publisher.GET
  def _getPublisher(self, **kwargs) -> Publisher:
    """Getter-function for the ROS publisher."""
    if self.__inner_publisher__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPublisher()
      return self._getPublisher(_recursion=True)
    if isinstance(self.__inner_publisher__, Publisher):
      return self.__inner_publisher__
    e = typeMsg('__inner_publisher__', self.__inner_publisher__, Publisher)
    raise TypeError(e)

  @topicName.GET
  def _getTopicName(self) -> str:
    """Getter-function for topic name"""
    if isinstance(self.__topic_name__, str):
      return self.__topic_name__
    e = typeMsg('__topic_name__', self.__topic_name__, str)
    raise TypeError(e)

  def _publish(self) -> None:
    """Publisher-function for the ROS message."""
    if isinstance(self.publisher, Publisher):
      self.publisher.publish(self._getCommand())
      self.published.emit(self.__inner_state__)

  def __init__(self, topicName: str) -> None:
    QThread.__init__(self)
    topicType = resolveTopicType(topicName)
    if not topicType == AuxCommand:
      e = """Expected topic type to be AuxCommand, but received: %s!"""
      raise ValueError(monoSpace(e % topicType))
    self.__topic_name__ = topicName

  @Slot()
  def start(self, ) -> None:
    """Slot for changing the signal state to True"""
    if not self.state:
      self.state = True
      self.stateChanged.emit(True)

  @Slot()
  def stop(self, ) -> None:
    """Slot for changing the signal state to False"""
    if self.state:
      self.state = False
      self.stateChanged.emit(False)

  def run(self):
    if isinstance(self.timer, QTimer):
      self.timer.start()
