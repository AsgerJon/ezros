"""ChartWidget provides visual representations of live data."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any, Never

from PySide6.QtCharts import QChartView, QChart
from PySide6.QtCore import QPointF, Slot
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FloatField
from vistutils.waitaminute import typeMsg

from ezros.rosutils import ArrayField, ListField


class ChartWidget(BaseWidget):
  """The ChartWidget class provides visual representations of live data."""

  __data_chart__ = None
  __data_series__ = None
  __data_axis_x__ = None
  __data_axis_y__ = None

  baseLayout = BaseLayoutField(layout='horizontal')
  data = ArrayField(128)
  pointLimit = FloatField(1024)

  minEdge = FloatField(-1.5)
  minLim = FloatField(-1.0)
  maxLim = FloatField(1.0)
  maxEdge = FloatField(1.5)

  epochTime = FloatField(1.0)
  values = ListField(QPointF)

  @Slot(float)
  def appendValue(self, value: float) -> None:
    """Append a value to the data array."""
    while len(self.values) >= self.pointLimit:
      self.values.pop(0)
    self.values.append(QPointF(time.time(), value))

  def getTimeRange(self, ) -> tuple[float, float]:
    """Returns the range of the data."""
    return time.time() - self.epochTime, time.time()

  def getValueRange(self, ) -> tuple[float, float]:
    """Returns the range of the data."""
    return self.minEdge, self.maxEdge

  def initUI(self) -> None:
    """Initializes the user interface."""
    self.baseLayout.addWidget(self.getView())
    self.setLayout(self.baseLayout)

  def getChart(self, ) -> QChart:
    """Returns the chart."""

  def getView(self, ) -> QChartView:
    """Returns the chart view."""
    view = QChartView()
    view.setChart(self.getChart())
    return view
