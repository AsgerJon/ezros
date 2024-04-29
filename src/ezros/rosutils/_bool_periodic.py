"""BoolPeriodic provides a subclass of BoolPub that turns a signal on and off
according to a specified program. This program is defined by an on period
and an off period that repeats."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from attribox import AttriBox
from ezside.core import EZTimer
from genpy import Duration
from rospy import Time
from vistutils.waitaminute import typeMsg

from ezros.rosutils import BoolPub


class BoolPeriodic(BoolPub):
  """BoolPeriodic provides a subclass of BoolPub that periodically turns
  a signal on or off."""

  __low_epoch__ = None
  __high_epoch__ = None
  __low_fallback__ = None
  __high_fallback__ = None

  @staticmethod
  def _validateTime(period: Any) -> float:
    """Validate the time."""
    out = None
    if isinstance(period, Duration) and out is None:
      out = period.to_sec()
    elif isinstance(period, int):
      out = float(period)
    elif isinstance(period, float):
      out = period
    if out is None:
      e = typeMsg('time', period, float)
      raise TypeError(e)
    if out > 0:
      return out
    e = """The period must be a positive number!"""
    raise ValueError(e)

  def _setLowEpoch(self, period: Any) -> None:
    """Setter-function for the duration of low signal"""
    self.__low_epoch__ = self._validateTime(period)

  def _getLowEpoch(self, **kwargs) -> float:
    """Getter-function for the duration of low signal"""
    if self.__low_epoch__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._setLowEpoch(self.__low_fallback__)
      return self._getLowEpoch(_recursion=True)
    return self._validateTime(self.__low_epoch__)

  def _setHighEpoch(self, period: Any) -> None:
    """Setter-function for the duration of high signal"""
    self.__high_epoch__ = self._validateTime(period)

  def _getHighEpoch(self, **kwargs) -> float:
    """Getter-function for the duration of high signal"""
    if self.__high_epoch__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._setHighEpoch(self.__high_fallback__)
      return self._getHighEpoch(_recursion=True)
    return self._validateTime(self.__high_epoch__)

  def getValue(self) -> bool:
    """Return the value of the signal."""
    period = self._getLowEpoch() + self._getHighEpoch()
    rightNow = Time().to_nsec() % (period * 1e09)
    if rightNow < self._getLowEpoch() * 1e09:
      return False
    return True
