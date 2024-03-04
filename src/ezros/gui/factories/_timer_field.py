"""TimerField provides a descriptor implementation for QTimer objects."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from vistutils.text import stringList

from ezros.gui.shortnames import Precise
from _dep.morevistutils import Field


class TimerField(Field):
  """TimerField provides a descriptor implementation for QTimer objects."""

  __epoch_length__ = None
  __single_shot__ = None
  __fallback_shot__ = False
  __timer_type__ = None
  __positional_args__ = None
  __keyword_args__ = None
  __fallback_type__ = Precise

  epochLength = Field()

  def __init__(self, *args, **kwargs) -> None:
    leftArgs, leftKwargs = [], {}
    epoch, shot, timerType = None, None, None
    epochKeys = stringList("""epoch, length, interval, time""")
    shotKeys = stringList("""shot, singleShot""")
    notShotKeys = stringList("""repeat, again""")
    timerTypeKeys = stringList("""timerType, precision, accuracy""")
    for (key, val) in kwargs.items():
      if key in epochKeys and epoch is None:
        if isinstance(val, int):
          epoch = val
      elif key in shotKeys and shot is None:
        shot = True if val else False
      elif key in notShotKeys and shot is None:
        shot = False if val else True
      elif key in timerTypeKeys and timerType is None:
        if isinstance(val, Qt.TimerType):
          timerType = val
      else:
        leftKwargs[key] = val
    for arg in args:
      if isinstance(arg, int) and epoch is None:
        epoch = arg
      if isinstance(arg, bool) and shot is None:
        shot = arg
      if isinstance(arg, Qt.TimerType) and timerType is None:
        timerType = arg
      else:
        leftArgs.append(arg)
    Field.__init__(self, *leftArgs, **leftKwargs)
    self.__positional_args__ = leftArgs
    self.__keyword_args__ = leftKwargs

  def _createTimer(self, instance) -> QTimer:
    """Creates a QTimer instance."""
    interval = self.__epoch_length__
    if interval is None:
      interval = 100
    timer = QTimer(instance)
    timer.setInterval(interval)
    timer.setTimerType(self.__timer_type__)
    timer.setSingleShot(self.__single_shot__)
    return timer
