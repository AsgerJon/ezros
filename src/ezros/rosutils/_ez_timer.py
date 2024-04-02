"""EZTimer wraps the QTimer class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QTimer, Qt, Signal
from ezside.core import Precise
from vistutils.parse import maybe


class EZTimer(QTimer):
  """EZTimer wraps the QTimer class"""

  __fallback_interval__ = 100
  __fallback_precision__ = Precise
  __fallback_single_shot__ = False

  __timeout_interval__ = None
  __timer_precision__ = None
  __single_shot__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the timer with the given interval and callback."""
    intervalKwarg = kwargs.get('interval', None)
    precisionKwarg = kwargs.get('precision', None)
    singleShotKwarg = kwargs.get('singleShot', None)
    intervalArg, precisionArg = None, None
    for arg in args:
      if isinstance(arg, int) and intervalArg is None:
        intervalArg = arg
      elif isinstance(arg, Qt.TimerType) and precisionArg is None:
        precisionArg = arg
    interval = maybe(intervalKwarg, intervalArg, self.__fallback_interval__)
    precision = maybe(precisionKwarg,
                      precisionArg,
                      self.__fallback_precision__)
    singleShot = maybe(singleShotKwarg, self.__fallback_single_shot__)
    QTimer.__init__(self)
    self.setInterval(interval)
    self.setTimerType(precision)
    self.setSingleShot(singleShot)
