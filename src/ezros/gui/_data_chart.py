"""DataChart provides a chart of live data"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCharts import QScatterSeries, QChart, QChartView
from PySide6.QtCore import QPointF, QEvent
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QVBoxLayout
from icecream import ic
from vistside.widgets import BaseWidget

from ezros.rosutils import ListField


class View(QChartView):
  """View provides a view with hooks for the data chart."""

  def showEvent(self, event: QEvent) -> None:
    """Show the event."""
    QChartView.showEvent(self, event)


class DataChart(BaseWidget):
  """DataChart provides a chart of live data"""

  data = ListField(tuple)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the DataChart."""
    BaseWidget.__init__(self, *args, **kwargs)
    self.series = QScatterSeries()
    self.chart = QChart()
    self.chart.addSeries(self.series)
    self.chart.createDefaultAxes()
    self.chart.axes()[0].setRange(0, 10)
    self.chart.axes()[1].setRange(-1, 1)
    self.view = View(self.chart)
    self.view.setRenderHint(QPainter.Antialiasing)
    self.view.setChart(self.chart)
    self.baseLayout = None
    self._zeroTime = time.time()

  def appendValue(self, value: float) -> None:
    """Append a value to the data array."""
    rightNow = time.time() - self._zeroTime
    point = QPointF(time.time() - self._zeroTime, value)
    self.series.append(point)
    # self.chart.axes()[0].setRange(rightNow - 10, rightNow)
    while self.series.count() > 128:
      self.series.removePoints(0, 1)
      self._zeroTime = self.series.at(0).x()

  def initUi(self, ) -> None:
    """Initializes the user interface."""
    self.baseLayout = QVBoxLayout()
    self.baseLayout.addWidget(self.view)
    self.setLayout(self.baseLayout)
