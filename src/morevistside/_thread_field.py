"""ThreadField provides a descriptor class for subclasses of QThread that
exist as instance variables in the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Type, Any, Never

from PySide6.QtCore import QThread
from vistutils.text import monoSpace
from vistutils.fields import AbstractField
from vistutils.waitaminute import typeMsg

ThreadType = Type[QThread]

__threads__ = []


class ThreadField(AbstractField):
  """ThreadField provides a descriptor class for subclasses of QThread that
  exist as instance variables in the main application window."""

  __thread_class__ = None

  def __prepare_owner__(self, owner: type) -> type:
    """Prepare the owner class for the field."""
    return owner

  def __init__(self, threadCls: ThreadType, *args) -> None:
    AbstractField.__init__(self)
    self._setThreadClass(threadCls)

  def _getThreadClass(self, ) -> ThreadType:  # ShibokenType?
    """Getter-function for the thread class"""
    return self.__thread_class__

  def _setThreadClass(self, cls: ThreadType) -> None:
    """Setter-function for the thread class"""
    if self.__thread_class__ is not None:
      raise AttributeError('The thread class is immutable!')
    if not isinstance(cls, type):
      raise TypeError('The thread class must be a type!')
    if not issubclass(cls, QThread):
      raise TypeError('The thread class must be a subclass of QThread!')
    self.__thread_class__ = cls

  def _threadFactory(self, instance: Any, ) -> QThread:
    """Factory function for creating the thread"""
    threadCls = self._getThreadClass()
    if instance is None:
      e = """Cannot create thread instance on the owner class itself!"""
      raise AttributeError(monoSpace(e))
    return threadCls(instance)

  def _createThread(self, instance: Any, ) -> None:
    """Creator function for the thread instance"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      e = """Thread instance already exists!"""
      raise AttributeError(monoSpace(e))
    thread = self._threadFactory(instance)
    setattr(instance, pvtName, thread)

  def _getThread(self, instance: Any, **kwargs) -> QThread:
    """Getter-function for the thread instance"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      thread = getattr(instance, pvtName)
      if isinstance(thread, self._getThreadClass()):
        return thread
      e = typeMsg('thread', thread, self._getThreadClass())
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self._createThread(instance)
    return self._getThread(instance, _recursion=True)

  def _setThread(self, *_) -> Never:
    """Illegal setter function"""
    clsName = self._getThreadClass().__name__
    e = """Instances of '%s' are read only!""" % clsName
    raise TypeError(monoSpace(e))

  def __get__(self, instance: Any, owner: type, **kwargs) -> QThread:
    """Getter-function for the field"""
    if instance is None:
      e = """Only instances provide a thread, not the class itself!"""
      raise AttributeError(monoSpace(e))
    return self._getThread(instance, **kwargs)

  def __set__(self, instance: Any, value: Any, **kwargs) -> Never:
    """Illegal setter function"""
    e = """The thread instance is immutable!"""
    raise TypeError(monoSpace(e))

  def __delete__(self, instance: Any) -> Never:
    """Illegal deleter function"""
    e = """The thread instance is immutable!"""
    raise TypeError(monoSpace(e))
