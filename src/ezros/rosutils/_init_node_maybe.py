"""
Initializes a ROS node if it hasn't been initialized yet with the
given name, or returns the caller ID of the already initialized node.
Raises an enhanced ROSInitException if initialization fails due to
improper arguments.

Args:
  nodeName: The name of the node to initialize.
  **kwargs: Additional keyword arguments for node initialization.

Returns:
  The name of the initialized node or the caller ID of the already
  initialized node.

Raises:
  ROSInitException: If the node initialization fails due to improper
  arguments, with a detailed explanation.
"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import get_caller_id, init_node, ROSException, ROSInitException


def initNodeMaybe(nodeName: str = None, **kwargs) -> str:
  """
  Initializes a ROS node if it hasn't been initialized yet with the
  given name, or returns the caller ID of the already initialized node.
  Raises an enhanced ROSInitException if initialization fails due to
  improper arguments.

  Args:
    nodeName: The name of the node to initialize.
    **kwargs: Additional keyword arguments for node initialization.

  Returns:
    The name of the initialized node or the caller ID of the already
    initialized node.

  Raises:
    ROSInitException: If the node initialization fails due to improper
    arguments, with a detailed explanation.
  """
  try:
    init_node(nodeName, **kwargs)
    return nodeName
  except ROSInitException as rosInitException:
    e = """When attempting to initiate the node, the following error of 
    the class ROSInitException was raised: This indicates that it was 
    appropriate to initiate the node, but that it was done with improper 
    arguments."""
    raise rosInitException from NameError(e)
  except ROSException as rosException:
    if """has already been called with different""" in str(rosException):
      return get_caller_id()
    raise rosException
