"""LiveData provides direct protection against evil garbage collectors."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QChartView
from PySide6.QtCore import Signal, Slot
from vistutils.text import monoSpace, stringList
from vistutils.waitaminute import typeMsg

from ezros.rosutils import RollingArray


class LiveData(QChartView):
  """LiveData provides direct protection against evil garbage collectors."""

  __rolling_array__ = None
  __default_num_points__ = None
  __scatter_series__ = None
  __line_series__ = None

  receivedValue = Signal(float)

  def __init__(self, *args, **kwargs) -> None:
    QChartView.__init__(self, *args, **kwargs)
    numKeys = stringList("""numPoints, num, n, size, length""")
    for key in numKeys:
      if key in kwargs:
        val = kwargs.get(key)
        if isinstance(val, int):
          self.__default_num_points__ = val
          break
        e = typeMsg(key, val, int)
        raise TypeError(e)
    else:
      for arg in args:
        if isinstance(arg, int):
          self.__default_num_points__ = arg
          break

  def _createRollingArray(self) -> None:
    """Create a rolling array."""
    if self.__rolling_array__ is None:
      if self.__default_num_points__ is None:
        self.__rolling_array__ = RollingArray()
      if isinstance(self.__default_num_points__, int):
        self.__rolling_array__ = RollingArray(self.__default_num_points__)
      else:
        e = typeMsg('defaultNumPoints', self.__default_num_points__, int)
        raise TypeError(e)
    else:
      e = """Creator function for rolling array instance called, but the 
      proper instance already exists. """
      raise RuntimeError(monoSpace(e))

  def _getRollingArray(self, **kwargs) -> RollingArray:
    """Get the rolling array."""
    if self.__rolling_array__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createRollingArray()
      return self._getRollingArray(_recursion=True)
    if isinstance(self.__rolling_array__, RollingArray):
      return self.__rolling_array__
    e = typeMsg('rollingArray', self.__rolling_array__, RollingArray)
    raise TypeError(e)
  
  def append(self, value: float) -> float:
    """Append a value to the rolling array."""
    self._getRollingArray().append(value)
    self.receivedValue.emit(value)
    return value

  @Slot()
  def visualUpdate(self) -> None:
    """Update the visual representation."""
