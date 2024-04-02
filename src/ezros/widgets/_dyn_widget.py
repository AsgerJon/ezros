"""DynWidget provides a widget view of the live updating chart."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from random import random

from PySide6.QtCharts import QChart, \
  QChartView, \
  QScatterSeries, \
  QLineSeries, \
  QValueAxis
from PySide6.QtCore import Slot, QPointF, Qt
from attribox import AttriBox
from ezside.widgets import BaseWidget
from icecream import ic
from msgs.msg import Float32Stamped
import numpy as np
from vistutils.parse import maybe

from ezros.defaults import Settings
from ezros.rosutils import RollingArray
from ezros.widgets import Vertical

ic.configureOutput(includeContext=True)


class DynWidget(BaseWidget):
  """DynWidget provides a widget view of the live updating chart."""

  __ros_publisher__ = None
  __right_now__ = None

  baseLayout = AttriBox[Vertical]()

  def __init__(self, numPoints: int = None) -> None:
    BaseWidget.__init__(self)
    self.chart = QChart()
    self.series = QLineSeries()
    self.chart.addSeries(self.series)

    # Axis configuration
    self.xAxis = QValueAxis()
    self.xAxis.setRange(0, 10)  # Set X axis range
    self.xAxis.setLabelFormat("%d")
    self.xAxis.setTitleText("Time")

    self.yAxis = QValueAxis()
    self.yAxis.setRange(0, 100)  # Set Y axis range
    self.yAxis.setLabelFormat("%d")
    self.yAxis.setTitleText("Value")

    self.chart.addAxis(self.xAxis, Qt.AlignmentFlag.AlignBottom)
    self.series.attachAxis(self.xAxis)
    self.chart.addAxis(self.yAxis, Qt.AlignmentFlag.AlignLeft)
    self.series.attachAxis(self.yAxis)

    self.chartView = QChartView(self.chart)

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.baseLayout.addWidget(self.chartView)
    self.setLayout(self.baseLayout)

  def refresh(self, array: np.ndarray) -> None:
    """Update the widget."""
    data = array.tolist()
    tMin, tMax = array.real.min(), array.real.max()
    # self.chart.axes()[0].setRange(tMin, tMax)
    # self.chart.axes()[1].setRange(-2, 2)
    self.series.clear()
    self.chart.removeSeries(self.series)
    for (i, item) in enumerate(data):
      if not i % 77:
        print(item.real, item.imag)
      self.series.append(QPointF(item.real, item.imag))
    self.chart.addSeries(self.series)
