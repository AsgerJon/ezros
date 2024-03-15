"""DataWidget provides a combined widget responsible for displaying data."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from types import FunctionType
from typing import Any

from PySide6.QtCharts import QChartView, QChart, QXYSeries
from PySide6.QtCore import Slot, QPointF, QEvent
from PySide6.QtGui import QPainter
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import IntField, unParseArgs, Wait

from ezros.rosutils import ListField


class HookedView(QChartView):
  """HookedView provides a view with hooks for the data widget."""

  #
  # hooks = ListField(FunctionType)
  #
  # def hook(self, callMeMaybe: FunctionType) -> None:
  #   """Hook the view."""
  #   self.hooks.append(callMeMaybe)
  #
  # def showHook(self, event: QEvent) -> None:
  #   """Show the hook."""
  #   for callMeMaybe in self.hooks:
  #     callMeMaybe(self, event)

  def showEvent(self, event: QEvent) -> None:
    """Show the event."""
    # self.showHook(event)
    QChartView.showEvent(self, event)


class PlotWidget(BaseWidget):
  """DataWidget provides a combined widget responsible for displaying
  data."""

  __chart_view__ = None
  __data_chart__ = None
  __data_series__ = None

  pointLimit = IntField(1024)
  baseLayout = BaseLayoutField()

  @Slot(float)
  def appendValue(self, value: float) -> None:
    """Append a value to the data array."""
    p = QPointF(time.time(), value)
    self._getSeries().append(p)
    while self._getSeries().count() > self.pointLimit:
      self._getSeries().removePoints(0, 100)

  def _createSeries(self, ) -> None:
    """Creates the series."""
    self.__data_series__ = QXYSeries()
    self.__data_series__.setName('Data')
    self.__data_series__.setUseOpenGL(True)

  def _getSeries(self, **kwargs) -> QXYSeries:
    """Returns the series."""
    if self.__data_series__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createSeries()
      return self._getSeries(_recursion=True)
    return self.__data_series__

  def _createChart(self, ) -> None:
    """Creates the chart."""
    self.__data_chart__ = QChart()
    series = self._getSeries()
    self.__data_chart__.addSeries(series)

  def _getChart(self, **kwargs) -> QChart:
    """Returns the chart."""
    if self.__data_chart__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createChart()
      return self._getChart(_recursion=True)
    return self.__data_chart__

  def _createView(self) -> None:
    """Creates the view."""
    self.__chart_view__ = HookedView()
    self.__chart_view__.setRenderHint(QPainter.Antialiasing)
    chart = self._getChart()
    self.__chart_view__.setChart(chart)

  def _getView(self, **kwargs) -> HookedView:
    """Returns the view."""
    if self.__chart_view__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createView()
      return self._getView(_recursion=True)
    return self.__chart_view__

  def initUi(self, ) -> None:
    """Initializes the user interface."""
    view = self._getView()
    self.baseLayout.addWidget(view)
    self.setLayout(self.baseLayout)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> PlotWidget:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> PlotWidget:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class PlotField(Wait):
  """The ScatterField class provides a descriptor for instances of
  Scatter."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ScatterField."""
    Wait.__init__(self, PlotWidget, *args, **kwargs)
