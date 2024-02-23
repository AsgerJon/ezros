"""The initializeRos initializes the ROS environment."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from rospy import init_node
from vistutils.parse import maybe


def initializeRosNode(nodeName: str, uri: str = None, **kwargs) -> bool:
  """Initialize the ROS environment."""
  if kwargs.get('_recursion', False):
    raise RecursionError
  if uri is None:
    uriKwarg = kwargs.get('uri', None)
    uriEnv = os.environ.get('ROS_MASTER_URI', None)
    localHost = 'http://localhost:11311'
    uri = maybe(uriKwarg, uriEnv, localHost)
  os.environ['ROS_MASTER_URI'] = uri
  anonymousFlagKwarg = kwargs.get('anonymous', None)
  anonymousFlagEnv = os.environ.get('ROS_NAMESPACE', None)
  anonymousFlag = maybe(anonymousFlagKwarg, anonymousFlagEnv, False)
  init_node(nodeName, anonymous=anonymousFlag, )
