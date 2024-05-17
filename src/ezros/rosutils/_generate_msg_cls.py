"""The 'generateMsgCls' class reads the content of a msg file and attempts
to generate a corresponding Python class."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from genpy import Message
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveMsgType


def generateMsgCls(msg_file: str) -> type:
  """Generate a Python class from a ROS msg file.

  Args:
      msg_file (str): The path to the ROS msg file.

  Returns:
      str: The Python class as a string.
  """
  with open(msg_file, 'r') as file:
    lines = file.readlines()

  clsName = msg_file.split('/')[-1].split('.')[0]
  slots = []
  types = []

  for line in lines:
    line = line.strip()
    if not line:
      continue
    if line.startswith('#'):
      continue
    key, val = [*line.split(' '), None, None][:2]
    if key is None or val is None:
      continue
    if key in slots:
      e = """Received duplicate key: '%s' in message file: '%s'!"""
      raise KeyError(e % (key, clsName))
    valType = resolveMsgType(val)
    if valType is None:
      e = """For key: '%s', the type listed: '%s', which could not be 
      resolved to any previous message type!"""
      raise ValueError(e % (key, val))
    if not isinstance(valType, type):
      e = typeMsg('valType', valType, type)
      raise TypeError(e)
    slots.append(key)
    types.append(valType)

  def _getTypes(self, ) -> list[type]:
    """Getter-function for the types of the message slots. """
    return self._slot_types

  namespace = {
    '__annotations__': dict(zip(slots, types)),
    '__slots__'      : slots,
    '_slot_types'    : types,
    '_get_types'     : _getTypes
  }

  return type(clsName, (Message,), namespace)
