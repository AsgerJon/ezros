"""The PlotWidget class provides a widget for plotting data.  """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from PySide6.QtCharts import QScatterSeries, QChart, QChartView
from PySide6.QtGui import QPainter
from vistside.core import parseParent
from vistside.widgets import BaseWidget

from ezros.rosutils import Array


class PlotWidget(BaseWidget):
  """The PlotWidget class provides a widget for plotting data."""

  def __init__(self, *args, **kwargs) -> None:
    self.series = QScatterSeries()
    self.chart = QChart()
    self.chart.addSeries(self.series)
    self.chart.createDefaultAxes()
    self.view = QChartView()
    self.view.setRenderHint(QPainter.Antialiasing)
    self.view.setChart(self.chart)
    parent = parseParent(*args)
    BaseWidget.__init__(self, parent, )
