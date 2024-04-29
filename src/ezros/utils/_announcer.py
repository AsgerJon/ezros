"""The Announcer class provides contextually aware logging instances. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.fields import EmptyField
from vistutils.waitaminute import typeMsg


class MetaAnnouncer(type):
  """The MetaAnnouncer metaclass provides a contextually aware logging
  class."""

  __event_loggers__ = None
  eventLoggers = EmptyField()

  @eventLoggers.GET
  def _getEventLoggers(cls, **kwargs) -> list[Announcer]:
    """Getter-function for event loggers"""
    if cls.__event_loggers__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      cls.__event_loggers__ = []
      return cls._getEventLoggers(_recursion=True, )
    if isinstance(cls.__event_loggers__, list):
      for logger in cls.__event_loggers__:
        if not isinstance(logger, Announcer):
          e = typeMsg('logger', logger, Announcer)
          raise TypeError(e)
      return cls.__event_loggers__
    e = typeMsg('eventLoggers', cls.__event_loggers__, list)
    raise TypeError(e)

  def __call__(cls, *args, **kwargs):
    """Create a new instance of the class."""
    self = super().__call__(*args, **kwargs)
    cls._getEventLoggers().append(self)
    return self

  def __str__(cls) -> str:
    """String representation"""
    entries = '\n'.join([str(logger) for logger in cls._getEventLoggers()])
    return 'Event Loggers for %s class:\n%s' % (cls.__name__, entries)


class Announcer(metaclass=MetaAnnouncer):
  """The Announcer class provides contextually aware logging instances. """

  __event_log__ = None

  eventLog = EmptyField()

  @eventLog.GET
  def _getEventLog(self, **kwargs) -> list[str]:
    """Getter-function for event log"""
    if self.__event_log__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.__event_log__ = []
      return self._getEventLog(_recursion=True, )
    if isinstance(self.__event_log__, list):
      for event in self.__event_log__:
        if not isinstance(event, str):
          e = typeMsg('event', event, str)
          raise TypeError(e)
      return self.__event_log__
    e = typeMsg('eventLog', self.__event_log__, list)
    raise TypeError(e)

  def __init__(self, msg: str) -> None:
    self._getEventLog().append('INIT: %s' % msg)

  def info(self, msg: str) -> None:
    """Log an info message"""
    self._getEventLog().append('INFO: %s' % msg)

  def warn(self, msg: str) -> None:
    """Log a warning message"""
    self._getEventLog().append('WARN: %s' % msg)

  def error(self, msg: str) -> None:
    """Log an error message"""
    self._getEventLog().append('ERROR: %s' % msg)

  def __str__(self) -> str:
    return '\n'.join(self._getEventLog())

  def save(self, path: str) -> None:
    """Save the event log to a file"""
    with open(path, 'w') as file:
      file.write(str(self))

  def exit(self, res: int) -> None:
    """Exit the program"""
    self._getEventLog().append('EXIT: %d' % res)
    print(self)
