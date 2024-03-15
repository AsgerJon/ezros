"""DynPlot provides a dynamic widget visualizing data in real-time."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QChart, QChartView, QXYSeries
from PySide6.QtCore import Slot, QSize
from PySide6.QtWidgets import QWidget
from vistside.core import parseParent, Black
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FieldBox

from ezros.rosutils import Array


class DynPlot(QWidget):
  """The DynPlot class provides a dynamic widget visualizing data in
  real-time."""

  __fallback_num_points__ = 128
  baseLayout = BaseLayoutField(layout='vertical')

  array = FieldBox[Array](256)
  series = FieldBox[QXYSeries]()
  chart = FieldBox[QChart]()
  view = FieldBox[QChartView]()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new DynPlot."""
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
    self.setMinimumSize(QSize(480, 240))

  def initUI(self, ) -> None:
    """Initializes the user interface."""
    self.chart.addSeries(self.series)
    self.chart.createDefaultAxes()
    self.view.setChart(self.chart)
    self.baseLayout.addWidget(self.view)
    self.setLayout(self.baseLayout)
    self.chart.axes()[1].setRange(-1.2, 1.2)
    self.chart.legend().setBorderColor(Black)
    self.chart.setBackgroundRoundness(8)
    self.chart.setTheme(QChart.ChartTheme.ChartThemeBlueNcs)
    self.chart.setAnimationOptions(QChart.AnimationOption.SeriesAnimations)

  @Slot(float)
  def callback(self, value: float) -> None:
    """Appends the value to the data."""
    self.array.append(value)

  @Slot()
  def updateChart(self, ) -> None:
    """Updates the chart with the current data."""
    self.series.clear()
    X = self.array.snap().real.tolist()
    Y = self.array.snap().imag.tolist()
    for (x, y) in zip(X, Y):
      self.series.append(x, y)

  def showEvent(self, event) -> None:
    """Shows the widget."""
    self.updateChart()
    BaseWidget.showEvent(self, event)
