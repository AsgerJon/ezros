"""LiveView fuck the garbage collector"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QChartView, QChart
from PySide6.QtCore import Slot
from attribox import AttriBox
from ezside.widgets import LineSeries, ScatterSeries
from ezside.settings import Default

from ezros.rosutils import QAttriBox, RollingArray


class LiveView(QChartView):
  """LiveView fuck the garbage collector"""
  innerChart = AttriBox[QChart]()
  lineVals = AttriBox[LineSeries]()
  scatter = AttriBox[ScatterSeries]()
  rollator = AttriBox[RollingArray]()

  def getScatter(self) -> ScatterSeries:
    """Getter-function for transient instances that the garbage collectors
    won't clear out."""
    series = ScatterSeries()
    series.appendNp(*self.rollator.rightNow())
    return series

  def getLine(self) -> LineSeries:
    """Getter-function for transient instances that the garbage collectors
    won't clear out."""
    series = LineSeries()
    series.appendNp(*self.rollator.rightNow())
    return series

  def initUi(self) -> None:
    """Sets up the view"""
    self.setMinimumSize(Default.chartViewWidth, Default.chartViewHeight)
    self.initScatter()

  def initScatter(self, ) -> None:
    """Initializes the scatter series."""
    self.setChart(self.innerChart)
    self.innerChart.removeAllSeries()
    self.innerChart.addSeries(self.getScatter())
    self.innerChart.createDefaultAxes()
    self.update()

  def initLine(self) -> None:
    """Initializes the line series."""
    self.setChart(self.innerChart)
    self.innerChart.removeAllSeries()
    self.innerChart.addSeries(self.getLine())
    self.innerChart.createDefaultAxes()
    self.update()

  @Slot(float)
  def append(self, value: float) -> None:
    """Appends a value to the series."""
    self.scatter.appendValue(value)
    self.lineVals.appendValue(value)

  @Slot()
  def refresh(self) -> None:
    """Refreshes the data."""
    self.scatter.updateValues()
    self.lineVals.updateValues()
    self.update()
