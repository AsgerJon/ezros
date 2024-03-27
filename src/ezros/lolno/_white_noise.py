"""WhiteNoise generates white noise and transmits it as ros messages"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class WhiteNoise:
  """WhiteNoise generates white noise and transmits it as ros messages"""

  __topic_name__ = None
  __publish_rate__ = None

  def __init__(self, topicName: str, publishRate: float) -> None:
    self.__topic_name__ = topicName
    self.__publish_rate__ = publishRate
