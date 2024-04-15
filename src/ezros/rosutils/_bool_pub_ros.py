"""BoolPubRos provides a QThread-based publisher for ROS messages."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QThread, Signal
from msgs.msg import AuxCommand
from rospy import Time, Publisher, Duration
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
from ezros.settings import Defaults
from ezros.utils import EmptyField


class BoolPubRos(QThread):
  """BoolPubRos provides a QThread-based publisher for publishing a state
  that is either ON or OFF to a named ROS topic. Please note that this
  class supports only boolean topic types such as AuxCommand"""

  __inner_state__ = None
  __inner_publisher__ = None
  __topic_name__ = None
  __topic_type__ = None

  activated = Signal()
  deactivated = Signal()
  stateChanged = Signal(bool)

  topicName = EmptyField()
  publisher = EmptyField()

  @topicName.GET
  def _getTopicName(self) -> str:
    """Getter-function for topic name"""
    if isinstance(self.__topic_name__, str):
      return self.__topic_name__
    e = typeMsg('__topic_name__', self.__topic_name__, str)
    raise TypeError(e)

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

  def __init__(self, topicName: str) -> None:
    """Initializes the PubRos instance."""
    QThread.__init__(self)
    if resolveTopicType(topicName) is not AuxCommand:
      e = """The topic type must be AuxCommand, but received: '%s'!"""
      raise ValueError(monoSpace(e % topicName))
    self.__topic_name__ = topicName

  def _getMsg(self) -> AuxCommand:
    """Getter-function for message."""
    auxCommand = AuxCommand()
    auxCommand.activate = True if self else False
    auxCommand.when = Time.now()
    auxCommand.for_duration = Duration.from_sec(0.1)
    return auxCommand

  def activate(self) -> None:
    """Activates the publisher."""
    if not self.__inner_state__:
      self.__inner_state__ = True
      self.stateChanged.emit(True)
      self.activated.emit()

  def deactivate(self) -> None:
    """Deactivates the publisher."""
    if self.__inner_state__:
      self.__inner_state__ = False
      self.stateChanged.emit(False)
      self.deactivated.emit()

  def __bool__(self) -> bool:
    """Returns the current state of the publisher."""
    return True if self.__inner_state__ else False

  def run(self) -> None:
    """Runs the thread."""
    while isinstance(self.publisher, Publisher):
      self.publisher.publish(self._getMsg())
      self.msleep(50)
