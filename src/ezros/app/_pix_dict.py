"""PixDict provides a descriptor implementation of a QPixmap valued
dictionary. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class PixDict:
  """PixDict provides a descriptor implementation of a QPixmap valued
  dictionary. """

  def __init__(self, pixDict: dict) -> None:
    self.__pix_dict__ = pixDict

  def __get__(self, instance: object, owner: object) -> dict:
    return self.__pix_dict__

  def __set__(self, instance: object, value: dict) -> None:
    self.__pix_dict__ = value

  def __delete__(self, instance: object) -> None:
    del self.__pix_dict__ss
