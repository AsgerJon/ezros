#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen

from PySide6.QtCharts import QChart, QLineSeries, QChartView, QValueAxis
from PySide6.QtCore import QPointF, QTimer, QSize
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QVBoxLayout, QInputDialog
from ezside.core import Expand
from ezside.widgets import BaseWidget
from icecream import ic

ic.configureOutput(includeContext=True)


class ComplexDataChartWidget(BaseWidget):
  """
  A widget that plots complex values on a QChart, with the real part
  representing time and the imaginary part representing the measured
  value. The chart updates itself at fixed intervals independent of
  data updates.
  """

  def __init__(self) -> None:
    BaseWidget.__init__(self)
    self.setSizePolicy(Expand, Expand)
    self.data = []
    self._createChart()
    self._createSeries()
    self._createView()
    self._createXAxis()
    self._createYAxis()
    self.initUI()
    self.timer = QTimer(self)
    self.timer.timeout.connect(self.refreshChart)
    self.setMouseTracking(True)
    self.timer.start(100)  # Refresh rate in milliseconds

  def _createChart(self) -> None:
    """Create the chart for the widget."""
    self.chart = QChart()

  def _createSeries(self) -> None:
    """Create the series for the chart."""
    self.series = QLineSeries()

  def _createView(self) -> None:
    """Create the chart view."""
    self.view = QChartView(self.chart)

  def _createXAxis(self) -> None:
    """Create the X axis for the chart."""
    self.xAxis = QValueAxis()

  def _createYAxis(self) -> None:
    """Create the Y axis for the chart."""
    self.yAxis = QValueAxis()

  def initUI(self) -> None:
    """Initializes the user interface."""
    self.setMinimumSize(QSize(640, 480))
    self.series.setName("Complex Data")
    self.chart.addSeries(self.series)
    self.chart.setAxisX(self.xAxis, self.series)
    self.chart.setAxisY(self.yAxis, self.series)

    self.xAxis.setTitleText("Time")
    self.yAxis.setTitleText("Value")
    self.yAxis.setRange(-1, 1)  # Adjust as needed

    layout = QVBoxLayout()
    layout.addWidget(self.view)
    self.setLayout(layout)

  def addData(self, complexData: complex) -> None:
    """Add a complex data point to the chart"""
    if not self.data:
      return self.data.append(complexData)
    now = self.data[-1].real
    timeLimit = 5
    while self.data[0].real < now - timeLimit:
      self.data.pop(0)
    self.data.append(complexData)

  def refreshChart(self) -> None:
    """Refresh the chart"""
    self.series.clear()
    for item in self.data:
      self.series.append(QPointF(item.real, item.imag))
    if self.data:
      minX = min(item.real for item in self.data)
      maxX = max(item.real for item in self.data)
      self.xAxis.setRange(minX, maxX)

  def setMax(self, ) -> None:
    """Set the maximum value of the Y axis"""
    res = QInputDialog.getDouble(self, 'Set Maximum', 'Maximum Value', )
    xMax, test = res
    xMin = self.yAxis.min()
    if test:
      self.yAxis.setRange(xMin, xMax)
      self.chart.update()

  def setMin(self, ) -> None:
    """Set the minimum value of the Y axis"""
    res = QInputDialog.getDouble(self, 'Set Minimum', 'Minimum Value', )
    xMin, test = res
    xMax = self.yAxis.max()
    if test:
      self.yAxis.setRange(xMin, xMax)
      self.chart.update()

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Handle mouse release events"""
    if event.button() == 2:
      self.setMax()
      print('LOL')
    BaseWidget.mouseReleaseEvent(self, event)
