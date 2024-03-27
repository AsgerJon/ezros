"""Pic encapsulates the PIL-image, allowing systematic manipulation of the
image. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from PIL import Image


class Pic:
  """Pic encapsulates the PIL-image, allowing systematic manipulation of the
  image. """

  __image_path__ = None
  __pil_image__ = None

  def __init__(self, imagePath: str) -> None:
    self.__image_path__ = imagePath
    self.__pil_image__ = None

  def loadImage(self) -> None:
    """Loads the image as a PIL image."""
    self.__pil_image__ = Image.open(self.__image_path__)
