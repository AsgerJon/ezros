"""DynPlot provides a dynamic widget visualizing data in real-time."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Self

from PySide6.QtCharts import (QChart, QScatterSeries, QChartView,
                              QLineSeries, \
                              QXYSeries)
from PySide6.QtCore import Slot, QSize
from PySide6.QtWidgets import QWidget
from vistside.core import parseParent, Black
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import unParseArgs, Wait

from ezros.rosutils import ArrayField


class Chart(QChart):
  """Wrapper class providing closure and descriptor."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the Chart."""
    QChart.__init__(self)
    self.legend().setBorderColor(Black)
    self.setBackgroundRoundness(8)
    self.setTheme(QChart.ChartTheme.ChartThemeBrownSand)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    defVal = Chart()
    defVal.apply((args, kwargs))
    return defVal

  def apply(self, value: Any) -> Chart:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ChartField(Wait):
  """The ChartField class provides a descriptor for instances of Chart."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ChartField."""
    Wait.__init__(self, Chart, *args, **kwargs)


class ScatterData(QScatterSeries):
  """Wrapper class providing closure and descriptor."""

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    defVal = cls()
    defVal.apply((args, kwargs))
    return defVal

  def apply(self, value: Any) -> ScatterData:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class LineData(QLineSeries):
  """Wrapper class providing closure and descriptor."""

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    defVal = cls()
    defVal.apply((args, kwargs))
    return defVal

  def apply(self, value: Any) -> LineData:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class DataField(Wait):
  """The ScatterField class provides a descriptor for instances of
  Scatter."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ScatterField."""
    for arg in args:
      if isinstance(arg, QXYSeries):
        args = (arg,)
    Wait.__init__(self, ScatterData, *args, **kwargs)


class View(QChartView):
  """Wrapper class providing closure and descriptor."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the View."""
    parent = parseParent(*args)
    QChartView.__init__(self, parent)
    self.setRubberBand(QChartView.RubberBand.NoRubberBand)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    defVal = cls()
    defVal.apply((args, kwargs))
    return defVal

  def apply(self, value: Any) -> View:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ViewField(Wait):
  """The ViewField class provides a descriptor for instances of View."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ViewField."""
    Wait.__init__(self, View, *args, **kwargs)


class DynPlot(QWidget):
  """The DynPlot class provides a dynamic widget visualizing data in
  real-time."""

  __fallback_num_points__ = 128
  baseLayout = BaseLayoutField(layout='vertical')
  data = ArrayField(128)

  chart = ChartField()
  lineData = DataField()
  view = ViewField()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new DynPlot."""
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
    self.setMinimumSize(QSize(480, 240))
    self._zeroTime = time.time()

  def initUI(self, ) -> None:
    """Initializes the user interface."""
    self.chart.addSeries(self.lineData)
    self.chart.createDefaultAxes()
    self.chart.axes()[1].setRange(-1.2, 1.2)
    self.view.setChart(self.chart)
    self.baseLayout.addWidget(self.view)
    self.setLayout(self.baseLayout)

  @Slot(float)
  def callback(self, value: float) -> None:
    """Appends the value to the data."""
    self.data.append(value)

  @Slot()
  def updateChart(self, ) -> None:
    """Updates the chart with the current data."""
    self.lineData.clear()
    X = self.data.snap().real.tolist()
    Y = self.data.snap().imag.tolist()
    for (x, y) in zip(X, Y):
      self.lineData.append(x, y)

  def showEvent(self, event) -> None:
    """Shows the widget."""
    BaseWidget.showEvent(self, event)
    self.updateChart()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class DynPlotField(Wait):
  """The DynPlotField class provides a descriptor for instances of
  DynPlot."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the DynPlotField."""
    Wait.__init__(self, DynPlot, *args, **kwargs)
