"""The 'timerFactory' streamlines the creation of QTimer instances. The
objects returned by the factories are instances of QTimer ready to be
connected.

Arguments:
"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import QTimer, Qt
from vistutils.parse import maybe, maybeType, searchKey
from vistutils.waitaminute import typeMsg

from ezros.gui.shortnames import Precise


def timerFactory(funcName: str = None) -> Callable:
  """Creates a QTimer instance."""

  def callMeMaybe(*args, **kwargs) -> QTimer:
    """Creates a QTimer instance."""
    intervalArg = maybeType(int, *args)
    intervalKwarg = searchKey(int, 'interval', 'time', **kwargs)
    intervalDefault = 100
    interval = maybe(intervalArg, intervalKwarg, intervalDefault)
    timerType = None
    for arg in args:
      if isinstance(arg, Qt.TimerType):
        timerType = arg
    if timerType is None:
      timerType = kwargs.get('timerType', Precise)
    timer = QTimer()
    timer.setInterval(interval)
    timer.setTimerType(timerType)
    timer.setSingleShot(True if kwargs.get('singleShot', False) else False)
    return timer

  if funcName is not None:
    if isinstance(funcName, str):
      callMeMaybe.__name__ = funcName
      return callMeMaybe
    e = typeMsg('funcName', funcName, str)
    raise TypeError(e)
  return callMeMaybe
