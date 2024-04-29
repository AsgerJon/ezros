"""LiveData provides a live view of continuously updating data."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCharts import QScatterSeries, \
  QChart, \
  QChartView, \
  QValueAxis, \
  QLineSeries
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF, QRect
from PySide6.QtGui import QBrush, QPen, QColor
from PySide6.QtWidgets import QVBoxLayout, QGraphicsView, QGraphicsRectItem
from ezside.core import AlignLeft, AlignBottom
from ezside.widgets import BaseWidget
from msgs.msg import Float32Stamped

from ezros.defaults import Settings


class LiveData(BaseWidget):
  """LiveData provides a live view of continuously updating data."""
  __danger_shade__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the LiveData instance."""
    BaseWidget.__init__(self, *args, **kwargs)
    self._baseLayout = QVBoxLayout()
    self._data = []
    self._xAxis = None
    self._yAxis = None
    self._series = QScatterSeries()
    self._series.setMarkerSize(Settings.pumpCurrentMarkerSize)
    fillColor = self._series.color()
    self._series.setBorderColor(fillColor)
    self._chart = QChart()
    self._chart.setTheme(Settings.getPumpCurrentTheme())
    self._view = QChartView()
    self._timer = QTimer()
    self._timer.setInterval(25)
    self._timer.timeout.connect(self.updateChart)

  def initUi(self) -> None:
    """The initUi method initializes the user interface of the window."""
    self._xAxis = Settings.getPumpCurrentXAxis()
    self._yAxis = Settings.getPumpCurrentYAxis()
    self._chart.addSeries(self._series)
    self._chart.addAxis(self._xAxis, AlignBottom)
    self._chart.addAxis(self._yAxis, AlignLeft)
    self._series.attachAxis(self._xAxis)
    self._series.attachAxis(self._yAxis)
    self._view.setChart(self._chart)
    self._baseLayout.addWidget(self._view)
    self.setLayout(self._baseLayout)
    self._timer.start()

  def append(self, data: Float32Stamped) -> None:
    """Appends the given value"""
    while len(self._data) + 2 > Settings.maxNumPoints:
      self._data.pop(0)
    self._data.append(data.header.stamp.to_nsec() * 1e-09 + data.data * 1j)

  def newDangerShade(self) -> QGraphicsRectItem:
    """Plots only the rectangle containing the scene"""
    return Settings.getPumpCurrentDangerRect(self._view.chart().plotArea())

  def resetDangerShade(self) -> None:
    """Resets the danger shade."""
    self._view.scene().removeItem(self.__danger_shade__)
    self.__danger_shade__ = self.newDangerShade()

  def getDangerShade(self) -> QGraphicsRectItem:
    """Returns the danger shade."""
    if self.__danger_shade__ is None:
      self.resetDangerShade()
    return self.__danger_shade__

  def updateDangerShade(self) -> None:
    """Updates the danger shade."""
    self._view.scene().removeItem(self.__danger_shade__)
    self.__danger_shade__ = self.newDangerShade()
    self._view.scene().addItem(self.__danger_shade__)

  def updateChart(self) -> None:
    """Updates the chart."""
    if not self._data:
      return
    self._series.clear()
    rightNow = self._data[-1].real
    for item in self._data:
      age = rightNow - item.real
      if age < Settings.maxAge:
        self._series.append(-age, item.imag)
    self.updateDangerShade()
