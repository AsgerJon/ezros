"""The subscriberFactory function creates a subscriber object. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from rospy import Subscriber
from vistutils.parse import maybe

from ezros.rosutils import resolveTopicType, validateInitialized


def _parse(*args, **kwargs) -> dict:
  """Parses the arguments and keyword arguments."""
  topicKwarg = kwargs.get('topicName', None)
  cbKwarg = kwargs.get('callback', None)
  nodeKwarg = kwargs.get('node', None)
  topicArg, cbArg, nodeArg = None, None, None
  for arg in args:
    if isinstance(arg, str):
      if topicArg is None:
        topicArg = arg
      elif nodeArg is None:
        nodeArg = arg
    elif callable(arg) and cbArg is None:
      cbArg = arg
  topicName = maybe(topicKwarg, topicArg)
  if topicName is None:
    raise ValueError('No topic name provided')
  callback = maybe(cbKwarg, cbArg)
  if callback is None:
    raise ValueError('No callback provided')
  nodeName = maybe(nodeKwarg, nodeArg, 'Test')
  return dict(topicName=topicName, callback=callback, nodeName=nodeName)


def subscriberFactory(*args, **kwargs) -> Any:
  """Create a subscriber for a given topic and message type."""
  parsed = _parse(*args, **kwargs)
  topicName = parsed['topicName']
  callback = parsed['callback']
  nodeName = parsed['nodeName']
  validateInitialized(nodeName)
  msgType = resolveTopicType(topicName)
  return Subscriber(topicName, msgType, callback)
