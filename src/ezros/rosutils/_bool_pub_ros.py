"""BoolPubRos provides a QThread-based publisher for ROS messages."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QThread
from msg import AuxCommand
from std_msgs.msg import Bool
from vistutils.text import joinWords, monoSpace

from ezros.rosutils import resolveTopicType


class BoolPubRos(QThread):
  """BoolPubRos provides a QThread-based publisher for publishing a state
  that is either ON or OFF to a named ROS topic. Please note that this
  class supports only boolean topic types such as AuxCommand"""

  __inner_state__ = None
  __topic_name__ = None
  __topic_type__ = None

  @classmethod
  def _getBoolTypes(cls) -> list[type]:
    """Getter-function for list of supported boolean types."""
    return [AuxCommand, Bool]

  def __init__(self, topicName: str) -> None:
    """Initializes the PubRos instance."""
    QThread.__init__(self)
    self.__topic_name__ = topicName
    topicType = resolveTopicType(self.__topic_name__)
    if topicType in self._getBoolTypes():
      self.__topic_type__ = topicType
    else:
      e = """Expected topic type to be one of: %s, but received: %s!"""
      boolTypesStr = joinWords(*[t.__name__ for t in self._getBoolTypes()])
      raise ValueError(monoSpace(e % (boolTypesStr, topicType)))

  def _getTopicType(self) -> type:
    """Getter-function for topic type."""
    return self.__topic_type__
