"""BoolPub subclasses PubRos to provide a publisher for the special binary
case that uses the AuxCommand message type. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, TYPE_CHECKING
from genpy import Duration, Time, Message
from PySide6.QtCore import Signal
from vistutils.text import monoSpace

from ezros.rosutils import PubRos, resolveTopicType

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


class BoolPub(PubRos):
  """BoolPub subclasses PubRos to provide a publisher for the special binary
  case that uses the AuxCommand message type. """

  __internal_state__ = None
  __high_value__ = None
  __low_value__ = None

  deactivated = Signal()
  activated = Signal()
  signalChanged = Signal(bool)

  def __init__(self, topicName: str, ) -> None:
    """Initializes the BoolPub instance."""
    topicType = resolveTopicType(topicName)
    if topicType != AuxCommand:
      e = """BoolPub supports only topics of type '%s', but received '%s' 
      which uses type: '%s'!"""
      raise TypeError(monoSpace(e % (AuxCommand, topicName, topicType)))
    PubRos.__init__(self, topicName, AuxCommand)

  def getValue(self) -> AuxCommand:
    """Getter-function for the internal state."""
    low = AuxCommand(False, Time(), 1.5 * self.getDuration())
    high = AuxCommand(True, Time(), 1.5 * self.getDuration())
    return high if self.__internal_state__ else low

  def activate(self, ) -> None:
    """Changes the internal state to HIGH"""
    if self.__internal_state__:
      return
    self.__internal_state__ = True
    self.activated.emit()
    self.signalChanged.emit(True)

  def deactivate(self, ) -> None:
    """Changes the internal state to LOW"""
    if not self.__internal_state__:
      return
    self.__internal_state__ = False
    self.deactivated.emit()
    self.signalChanged.emit(False)
