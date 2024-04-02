"""LMAO"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from PySide6.QtCharts import QChart, QLineSeries, QChartView, QValueAxis
from PySide6.QtCore import QTimer, QPointF, Qt
from ezside.core import Tight, Expand
import numpy as np


class ComplexDataChartWidget(QWidget):
  """
  A widget that plots complex values on a QChart, with the real part
  representing time and the imaginary part representing the measured
  value. The chart updates itself at fixed intervals independent of
  data updates.
  """

  def __init__(self, updateIntervalMs: int = 1000) -> None:
    super().__init__()
    self.data = []  # Stores the complex data points
    self.initUI()
    self.initTimer(updateIntervalMs)
    self.setSizePolicy(Expand, Expand)

  def initUI(self) -> None:
    """
    Initializes the user interface, including the chart and its series.
    """
    self.chart = QChart()
    self.series = QLineSeries()
    self.chart.addSeries(self.series)

    # Set up the axes
    self.xAxis = QValueAxis()
    self.xAxis.setTitleText("Time")
    self.chart.addAxis(self.xAxis, Qt.AlignBottom)
    self.series.attachAxis(self.xAxis)

    self.yAxis = QValueAxis()
    self.yAxis.setRange(-1, 10)
    self.chart.addAxis(self.yAxis, Qt.AlignLeft)
    self.series.attachAxis(self.yAxis)

    self.chartView = QChartView(self.chart)
    self.chartView.setSizePolicy(Expand, Expand)
    layout = QVBoxLayout()
    layout.addWidget(self.chartView)
    self.setLayout(layout)

  def initTimer(self, intervalMs: int) -> None:
    """
    Initializes a QTimer to update the chart at a fixed interval.
    """
    self.timer = QTimer(self)
    # self.timer.timeout.connect(self.refreshChart)
    self.timer.start(intervalMs)

  def addData(self, complexData: complex) -> None:
    """
    Slot to add new complex data. The real part represents time,
    and the imaginary part represents the measured value.
    """
    self.data.append(complexData)

  def refreshChart(self, data: Any) -> None:
    """
    Refreshes the chart to plot the current data.
    """
    self.series.clear()
    for dataPoint in data:
      self.series.append(QPointF(dataPoint.real, dataPoint.imag))
    if data.tolist():
      self.xAxis.setRange(min(dp.real for dp in data),
                          max(dp.real for dp in data))
      # self.yAxis.setRange(min(dp.imag for dp in data),
      #                     max(dp.imag for dp in data))
