"""The 'timerFactory' streamlines the creation of QTimer instances. The
objects returned by the factories are instances of QTimer ready to be
connected.

Arguments:
"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import QTimer, Qt, QObject
from vistutils.parse import maybe, maybeType, searchKey
from vistutils.waitaminute import typeMsg

from ezros.gui.shortnames import Precise


def timerFactory(*args, **kwargs) -> Callable:
  """Creates a QTimer instance."""

  def callMeMaybe(instance: QObject, *args2, **kwargs2) -> QTimer:
    """Creates a QTimer instance."""
    if not isinstance(instance, QObject):
      e = typeMsg('instance', QObject, instance)
      raise TypeError(e)
    intervalArg = maybeType(int, *args2)
    intervalKwarg = searchKey(int, 'interval', 'time', **kwargs2)
    intervalDefault = 100
    interval = maybe(intervalArg, intervalKwarg, intervalDefault)
    timerType = None
    for arg in args2:
      if isinstance(arg, Qt.TimerType):
        timerType = arg
    if timerType is None:
      timerType = kwargs2.get('timerType', Precise)
    timer = QTimer(instance)
    timer.setInterval(interval)
    timer.setTimerType(timerType)
    timer.setSingleShot(True if kwargs2.get('singleShot', False) else False)
    return timer

  return callMeMaybe
