"""DynPlot provides a dynamic widget visualizing data in real-time."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Self

from PySide6.QtCharts import (QChart, QScatterSeries, QChartView,
                              QLineSeries, \
                              QXYSeries, QAbstractSeries)
from PySide6.QtCore import Slot, QSize, QPointF, QRect, Qt, QSizeF, QRectF
from PySide6.QtGui import QPaintEvent, QPainter, QColor, QPen, QBrush
from PySide6.QtWidgets import QWidget
from icecream import ic
from vistside.core import parseParent, Black, Green, Red
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import unParseArgs, Wait, FloatField
from vistutils.waitaminute import typeMsg

from ezros.rosutils import ArrayField

PointColor = QXYSeries.PointConfiguration.Color


class Chart(QChart):
  """Wrapper class providing closure and descriptor."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the Chart."""
    QChart.__init__(self)
    self.legend().setBorderColor(Black)
    self.setBackgroundRoundness(8)
    self.setTheme(QChart.ChartTheme.ChartThemeBrownSand)
    self.setAnimationOptions(QChart.AnimationOption.NoAnimation)

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
  lowRange = FloatField(-1.5)
  highRange = FloatField(1.5)
  minVal = FloatField(0.)
  maxVal = FloatField(1.)

  def append(self, *args, **kwargs) -> None:
    """Appends a point to the series."""
    if len(args) == 1:
      if isinstance(args[0], QPointF):
        point = args[0]
      elif isinstance(args[0], complex):
        point = QPointF(args[0].real, args[0].imag)
      else:
        return QScatterSeries.append(self, *args, **kwargs)
    else:
      floatArgs = [arg for arg in args if isinstance(arg, (float, int))]
      if len(floatArgs) == 2:
        point = QPointF(*[float(arg) for arg in floatArgs])
      else:
        return QScatterSeries.append(self, *args, **kwargs)

      index = self.count()
      QScatterSeries.append(self, point)
      if isinstance(point, QPointF):
        if self.minVal < point.y() < self.maxVal:
          self.setPointConfiguration(index, PointColor, Green)
        else:
          self.setPointConfiguration(index, PointColor, Red)

  @Slot()
  def setAir(self, ) -> None:
    """Sets the air."""
    self.minVal = -0.5
    self.maxVal = 0.5

  @Slot()
  def setWater(self, ) -> None:
    """Sets the water."""
    self.minVal = -0.75
    self.maxVal = 0.75

  @Slot()
  def setPaint(self, ) -> None:
    """Sets the paint."""
    self.minVal = -1.0
    self.maxVal = 1.0

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    defVal = cls()
    defVal.apply((args, kwargs))
    return defVal

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class DataField(Wait):
  """The ScatterField class provides a descriptor for instances of
  Scatter."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ScatterField."""
    Wait.__init__(self, ScatterData, *args, **kwargs)


class View(QChartView):
  """Wrapper class providing closure and descriptor."""

  __explicit_chart__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the View."""
    parent = parseParent(*args)
    QChartView.__init__(self, parent)
    self.setRubberBand(QChartView.RubberBand.NoRubberBand)
    self.lmao = True

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
    self.lineData.setAir()
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
    X = self.data.snap().real.tolist()
    Y = self.data.snap().imag.tolist()
    n = len(X)
    p0 = None
    if self.lineData.count():
      p0 = self.lineData.at(n)
      if not isinstance(p0, QPointF):
        e = typeMsg('p0', p0, QPointF)
        raise TypeError(e)
      self.lineData.clear()
      self.lineData.append(p0.x(), p0.y())
    for (x, y) in zip(X, Y):
      self.lineData.append(x, y)
    if isinstance(p0, QPointF):
      self.lineData.append(p0.x(), p0.y())
    self.lineData.setPointsVisible(True)

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
