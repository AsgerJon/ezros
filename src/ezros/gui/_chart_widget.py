"""ChartWidget provides visual representations of live data."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Never

from PySide6.QtCharts import QChartView, QChart
from PySide6.QtCore import QPointF, Slot, QEvent
from vistside.core import Bottom, Left
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FloatField
from vistutils.waitaminute import typeMsg

from ezros.gui import DataField, AxisField, ChartField, ViewField
from ezros.rosutils import ListField


class ChartWidget(BaseWidget):
  """The ChartWidget class provides visual representations of live data."""

  __data_chart__ = None
  __data_series__ = None
  __data_axis_x__ = None
  __data_axis_y__ = None

  baseLayout = BaseLayoutField(layout='horizontal')
  pointLimit = FloatField(1024)

  minEdge = FloatField(-1.5)
  minLim = FloatField(-1.0)
  maxLim = FloatField(1.0)
  maxEdge = FloatField(1.5)

  epochTime = FloatField(1.0)
  data = DataField()
  timeAxis = AxisField()
  valueAxis = AxisField()
  dataChart = ChartField()
  dataView = ViewField()

  @Slot(float)
  def appendValue(self, value: float) -> None:
    """Append a value to the data array."""
    while len(self.data) >= self.pointLimit:
      self.data.pop(0)
    self.data.append(QPointF(time.time(), value))

  def getTimeRange(self, ) -> tuple[float, float]:
    """Returns the range of the data."""
    return time.time() - self.epochTime, time.time()

  def getValueRange(self, ) -> tuple[float, float]:
    """Returns the range of the data."""
    return self.minEdge, self.maxEdge

  def initUI(self) -> None:
    """Initializes the user interface."""
    self.setupView()
    self.baseLayout.addWidget(self.dataView)
    self.setLayout(self.baseLayout)

  def setupView(self) -> None:
    """Sets up the view."""
    self.dataChart.addSeries(self.data)
    self.data.attachAxis(self.timeAxis)
    self.data.attachAxis(self.valueAxis)
    self.dataChart.addAxis(self.timeAxis, Bottom)
    self.dataChart.addAxis(self.valueAxis, Left)
    self.dataView.setChart(self.dataChart)

  def showEvent(self, event: QEvent) -> None:
    """Handles the show event."""
    self.setupView()
    BaseWidget.showEvent(self, event)
