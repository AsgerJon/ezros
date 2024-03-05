"""NodeField ensures that the process has a node available."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QObject
from rospy import init_node, get_node_uri
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from _dep.morevistutils import parseName

ShibokenType = type(QObject)


class NodeField:
  """NodeField ensures that the process has a node available."""

  __field_name__ = None
  __field_owner__ = None
  __node_name__ = None
  __anonymous_flag__ = False

  @staticmethod
  def _nodeStatus() -> bool:
    """Returns True if the node is initialized."""
    return False if get_node_uri() is None else True

  @staticmethod
  def __init__(self, *args, **kwargs) -> None:
    self.__node_name__ = parseName(*args, **kwargs)
    self.__anonymous_flag__ = kwargs.get('anonymous', False)

  def _wrapInitNode(self) -> None:
    """Initializes the node."""
    if not self._nodeStatus():
      init_node(self.__node_name__, anonymous=self.__anonymous_flag__)

  def __bool__(self, ) -> bool:
    """Reflects the node status"""
    return self._nodeStatus()

  def __str__(self, ) -> str:
    """Returns the node name"""
    if self:
      msg = """A node named '%s' is running through uri: '%s' at this 
      process. """ % (self.__node_name__, get_node_uri())
      return monoSpace(msg)
    return """No node is running at this process."""

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the field name and owner"""
    if not type(owner) is ShibokenType:
      e = typeMsg('owner', owner, ShibokenType)
      raise TypeError(e)

    self.__field_name__ = name
    self.__field_owner__ = owner
    if self.__node_name__ is None:
      self.__node_name__ = name
    setattr(owner, '__node_name__', self.__node_name__)
    originalInit = getattr(owner, '__init__', None)
    if originalInit is object.__init__:
      return setattr(owner, '__init__', self._wrapInitNode)

    def wrappedInit(*args, **kwargs) -> None:
      """Wraps the __init__ method"""
      self._wrapInitNode()
      return originalInit(*args, **kwargs)

    setattr(owner, '__init__', wrappedInit)
