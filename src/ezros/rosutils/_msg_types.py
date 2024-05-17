"""This file provides functions for retrieving existing msg types."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

import std_msgs.msg
from vistutils.waitaminute import typeMsg


def getMsgTypes() -> dict[str, type]:
  """Get all existing msg types in the ROS workspace.

  Returns:
    dict[str, type]: A dictionary with the names of all existing msg types as
    keys and the corresponding msg type classes as values.
  """
  out = {}
  for (key, val) in std_msgs.msg.__dict__.items():
    if isinstance(val, type) and not key.startswith("_"):
      out[key] = val
  return out


def getMsgTypeNames() -> list[str]:
  """Get the names of all existing msg types in the ROS workspace.

  Returns:
    list[str]: The names of all existing msg types.
  """
  return [key for (key, _) in getMsgTypes().items()]


def resolveMsgType(msgTypeName: str) -> Optional[type]:
  """Resolve a msg type by its name."""
  if '/' in msgTypeName:
    for item in msgTypeName.split('/'):
      resolvedType = resolveMsgType(item)
      if resolvedType is not None:
        if isinstance(resolvedType, type):
          return resolvedType
        e = typeMsg('resolvedType', resolvedType, type)
        raise TypeError(e)
    else:
      return None
  types = getMsgTypes()
  primitiveTypes = dict(int=std_msgs.msg.Int64,
                        float=std_msgs.msg.Float32,
                        str=std_msgs.msg.String,
                        bool=std_msgs.msg.Bool)
  return {**types, **primitiveTypes}.get(msgTypeName, None)
