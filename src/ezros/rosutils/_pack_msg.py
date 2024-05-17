"""PackMsg provides a decorator indicating that the decorated class
should be packed into a ROS message type. The decorator should be
instantiated with the name of the ros package to which the generated msg
type should be added. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class PackMsg:
  """PackMsg provides a decorator indicating that the decorated class
  should be packed into a ROS message type. The decorator should be
  instantiated with the name of the ros package to which the generated msg
  type should be added. """

  __ros_package__ = None

  def __init__(self, rosPackage: str, *args, **kwargs) -> None:
    self.__ros_package__ = rosPackage
