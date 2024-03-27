"""Timer wrapper on QTimer"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QTimer
from ezside.core import TimerType, Precise
from ezside.widgets import BaseWidget
from icecream import ic
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

TimerArgs = tuple[int, TimerType, bool]

ic.configureOutput(includeContext=True)


class Timer(QTimer):
  """Timer wrapper on QTimer"""

  @staticmethod
  def typeGuard(interval: int,
                timerType: TimerType,
                singleShot: bool) -> TimerArgs:
    """Type guard"""
    if not isinstance(interval, int):
      e = typeMsg('interval', interval, int)
      raise TypeError(e)
    if not isinstance(timerType, TimerType):
      e = typeMsg('timerType', timerType, TimerType)
      raise TypeError(e)
    if not isinstance(singleShot, bool):
      e = typeMsg('singleShot', singleShot, bool)
      raise TypeError(e)
    return interval, timerType, singleShot

  @classmethod
  def parseThis(cls, *args, **kwargs) -> TimerArgs:
    """Looks for an instance of BaseWidget"""
    for arg in args:
      if isinstance(arg, BaseWidget):
        widget = arg
        break
    else:
      raise ValueError('No BaseWidget found')
    interval = getattr(widget, '__timer_interval__', None)
    timerType = getattr(widget, '__timer_type__', Precise)
    singleShot = getattr(widget, '__single_shot__', False)
    return cls.typeGuard(interval, timerType, singleShot)

  @classmethod
  def parseArgs(cls, *args, **kwargs) -> TimerArgs:
    """Parses the arguments"""
    interval = None
    timerType = None
    singleShot = None
    intervalKeys = stringList("""interval, time, epoch, period""")
    typeKeys = stringList("""timerType, type, mode""")
    singleShotKeys = stringList("""singleShot, oneShot, once""")
    Keys = [intervalKeys, typeKeys, singleShotKeys]
    for keys in Keys:
      for key in keys:
        if key in kwargs:
          val = kwargs.get(key)
          if key in intervalKeys:
            if isinstance(val, int):
              interval = val
              break
          elif key in typeKeys:
            if isinstance(val, int):
              timerType = val
              break
          elif key in singleShotKeys:
            if isinstance(val, bool):
              singleShot = val
              break
    else:
      for arg in args:
        if isinstance(arg, int):
          interval = arg
        if isinstance(arg, bool):
          singleShot = arg
        if isinstance(arg, TimerType):
          timerType = arg
      interval = maybe(interval, None)
      timerType = maybe(timerType, Precise)
      singleShot = maybe(singleShot, False)
    return cls.typeGuard(interval, timerType, singleShot)

  def __init__(self, *args, **kwargs) -> None:
    try:
      interval, timerType, singleShot = self.parseThis(*args, **kwargs)
    except ValueError as valueError:
      try:
        interval, timerType, singleShot = self.parseArgs(*args, **kwargs)
      except TypeError as typeError2:
        raise valueError from typeError2
    QTimer.__init__(self, )
    self.setInterval(interval)
    self.setTimerType(timerType)
    self.setSingleShot(singleShot)
    ic(interval)
