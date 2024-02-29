"""Sub instances represent a named topic in ROS. The instances can then
decorate"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class Sub:
  """Sub instances represent a named topic in ROS. The instances can then
  decorate"""

  __field_name__ = None
  __field_type__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    self.__field_name__ = name
    self.__field_type__ = owner

  def getTopicName(self) -> str:
    """Getter-function for the topic name"""
    return self.__field_name__
