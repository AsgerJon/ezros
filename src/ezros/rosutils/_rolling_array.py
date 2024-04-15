"""RollingArray to keep visual representation classes subservient to the
actual data structure classes. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCharts import QScatterSeries, QChart, QChartView, QValueAxis
from PySide6.QtCore import QTimer, Slot, Qt
from PySide6.QtWidgets import QGraphicsView
from attribox import AttriBox
from ezside.widgets import BaseWidget, Vertical
from icecream import ic
from msgs.msg import Float32Stamped
import numpy as np
from rospy import Time
from vistutils.waitaminute import typeMsg

from ezros.settings import Defaults
from ezros.utils import EmptyField

ic.configureOutput(includeContext=True)


class RollingArray(BaseWidget):
  """RollingArray to keep visual representation classes subservient to the
  actual data structure classes. """

  __max_num_points__ = Defaults.maxNumPoints

  __inner_data__ = None
  __inner_timer__ = None
  __inner_series__ = None
  __inner_chart__ = None
  __inner_view__ = None
  __iter_contents__ = None

  data = EmptyField()
  timer = EmptyField()
  baseLayout = AttriBox[Vertical]()
  series = AttriBox[QScatterSeries]()
  axisX = AttriBox[QValueAxis]()
  axisY = AttriBox[QValueAxis]()
  chart = AttriBox[QChart]()
  live = AttriBox[QChartView]()

  @data.GET
  def _getInnerData(self, **kwargs) -> list[complex]:
    """Getter-function for the inner data."""
    if self.__inner_data__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      kwargs['_recursion'] = True
      self.__inner_data__ = []
      return self._getInnerData(**kwargs)
    if isinstance(self.__inner_data__, list):
      rightNow = Time.now().to_nsec() * 1e-09
      return [arg - rightNow + 0j for arg in self.__inner_data__]
    e = typeMsg('__inner_data__', self.__inner_data__, list)
    raise TypeError(e)

  def _createTimer(self) -> None:
    """Creator-function for the timer."""
    self.__inner_timer__ = QTimer()
    self.__inner_timer__.setInterval(Defaults.chartUpdateInterval)
    self.__inner_timer__.setTimerType(Defaults.timerType)
    self.__inner_timer__.setSingleShot(False)
    self.__inner_timer__.timeout.connect(self.update)

  @timer.GET
  def _getTimer(self, **kwargs) -> QTimer:
    """Getter-function for the timer."""
    if self.__inner_timer__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createTimer()
      return self._getTimer(_recursion=True)
    if isinstance(self.__inner_timer__, QTimer):
      return self.__inner_timer__
    e = typeMsg('__inner_timer__', self.__inner_timer__, QTimer)
    raise TypeError(e)

  def __iter__(self, ) -> RollingArray:
    """Iterator for the RollingArray."""
    if isinstance(self.data, list):
      rightNow = Time.now()
      self.__iter_contents__ = [i for i in self.data]
      return self

  def __next__(self) -> complex:
    """Next method for the RollingArray."""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration

  def __len__(self, ) -> int:
    """Length method for the RollingArray."""
    if isinstance(self.data, list):
      return len(self.data)
    e = typeMsg('self.data', self.data, list)
    raise TypeError(e)

  def __bool__(self, ) -> bool:
    """Boolean method for the RollingArray."""
    if isinstance(self.data, list):
      return True if self.data else False
    e = typeMsg('self.data', self.data, list)
    raise TypeError(e)

  @Slot(complex)
  def append(self, dataPoint: complex) -> complex:
    """Appends and returns given data point"""
    self.__inner_data__.append(dataPoint)
    return dataPoint

  @Slot()
  def update(self) -> None:
    """Updates the chart."""
    ic('Updating chart: %d' % len(self.data))
    if not isinstance(self.data, list):
      e = typeMsg('self.data', self.data, list)
      raise TypeError(e)
    if not isinstance(self.series, QScatterSeries):
      e = typeMsg('self.series', self.series, QScatterSeries)
      raise TypeError(e)
    if not isinstance(self.chart, QChart):
      e = typeMsg('self.chart', self.chart, QChart)
      raise TypeError(e)
    if not isinstance(self.live, QChartView):
      e = typeMsg('self.live', self.live, QChartView)
      raise TypeError(e)
    # self.chart.removeSeries(self.series)
    self.series.clear()
    rightNow = time.time()
    for dataPoint in self.data:
      self.series.append(dataPoint.real - rightNow, dataPoint.imag)
      # print(dataPoint.real, dataPoint.imag)
    # self.chart.addSeries(self.series)
    # self.chart.createDefaultAxes()
    # self.chart.axes(Qt.Orientation.Vertical)
    # self.live.setChart(self.chart)
    # self.chart.update()
    self.live.update()
    BaseWidget.update(self)

  def initUi(self) -> None:
    """The initUi method initializes the user interface of the window"""
    self.axisX.setRange(-Defaults.maxAge, 0)
    self.axisY.setRange(-2, 2)
    self.chart.addAxis(self.axisX, Qt.AlignmentFlag.AlignBottom)
    self.chart.addAxis(self.axisY, Qt.AlignmentFlag.AlignLeft)
    self.chart.addSeries(self.series)
    self.live.setChart(self.chart)
    self.baseLayout.addWidget(self.live)
    self.setLayout(self.baseLayout)
    if isinstance(self.timer, QTimer):
      self.timer.start()
    else:
      e = typeMsg('self.timer', self.timer, QTimer)
      raise TypeError(e)
