"""The getNodeStatus function tests if given node is running."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from rosnode import get_node_names

ic.configureOutput(includeContext=True)


def getNodeStatus(nodeName: str = None) -> bool:
  """The getNodeStatus function tests if given node is running."""
  for name in get_node_names():
    if nodeName is None:
      return True
    if name in nodeName or nodeName in name:
      return True
  else:
    return False
