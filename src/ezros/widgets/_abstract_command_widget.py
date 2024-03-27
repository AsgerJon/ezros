"""AbstractCommandWidget provides a base class for widgets issuing start,
stop and specific AuxCommands to a particular topic. This abstract base
class provides only slots. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Slot, Signal
from attribox import this, AttriBox
from ezside.core import Precise
from ezside.widgets import BaseWidget
from icecream import ic
from msgs.msg import AuxCommand
from rospy import Duration, get_rostime

from ezros.rosutils import Commander
from ezros.widgets import Timer

ic.configureOutput(includeContext=True)


class AbstractCommandWidget(BaseWidget):
  """The AbstractCommandWidget class provides a base class for widgets
  issuing start, stop and specific AuxCommands to a particular topic. """

  __fallback_node_name__ = 'TestNode'

  __topic_name__ = None
  __node_name__ = None

  commander = AttriBox[Commander](this)
  activeTimer = AttriBox[Timer](500, Precise, False)

  started = Signal()
  stopped = Signal()
  receivedSpecial = Signal(AuxCommand)
  receivedTimed = Signal(float)

  @staticmethod
  def getStartCommand() -> AuxCommand:
    """Start the command."""
    return AuxCommand(True, get_rostime(), Duration.from_sec(3))

  @staticmethod
  def getStopCommand() -> AuxCommand:
    """Stop the command."""
    return AuxCommand(True, get_rostime(), Duration.from_sec(0.01))

  @staticmethod
  def getTimedCommand(duration: float) -> AuxCommand:
    """Get a timed command."""
    return AuxCommand(True, get_rostime(), Duration.from_sec(duration))

  @Slot()
  def start(self) -> None:
    """Start the command."""
    ic('start')
    self._explicitStart()
    self.activeTimer.start()
    self.started.emit()

  @Slot()
  def stop(self) -> None:
    """Start the command."""
    ic('stop')
    self._explicitStop()
    self.activeTimer.stop()
    self.stopped.emit()

  @Slot()
  def _explicitStart(self) -> None:
    """Explicitly starts the command."""
    self.commander.publish(self.getStartCommand())

  @Slot()
  def _explicitStop(self) -> None:
    """Explicitly stops the command."""
    self.commander.publish(self.getStopCommand())

  @Slot(float)
  def timed(self, duration: float) -> None:
    """Start the command."""
    self.commander.publish(self.getTimedCommand(duration))
    self.receivedTimed.emit(duration)

  @Slot(AuxCommand)
  def special(self, command: AuxCommand) -> None:
    """Start the command."""
    self.commander.publish(command)
    self.receivedSpecial.emit(command)

  def getNodeName(self) -> str:
    """Get the node name."""
    if self.__node_name__ is None:
      return self.__fallback_node_name__
    return self.__node_name__

  def getTopicName(self) -> str:
    """Get the topic name."""
    return self.__topic_name__

  def setTopicName(self, topicName: str) -> None:
    """Set the node name."""
    self.__topic_name__ = topicName
    self.commander.reset(topicName, )

  def connectActions(self) -> None:
    """Connect the actions."""
    self.activeTimer.timeout.connect(self._explicitStart)
