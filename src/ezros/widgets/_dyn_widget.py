"""DynWidget provides a widget view of the live updating chart."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QChart, QChartView, QScatterSeries
from PySide6.QtCore import Slot, QPointF
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
    self._series = QScatterSeries()
    self._chart = QChart()
    self._chart.addSeries(self._series)
    self._chart.createDefaultAxes()
    self._view = QChartView()
    self._view.setChart(self._chart)

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.baseLayout.addWidget(self._view)
    self.setLayout(self.baseLayout)

  def refresh(self, array: np.ndarray) -> None:
    """Update the widget."""
    data = array.tolist()
    tMin, tMax = array.real.min(), array.real.max()
    self._series.clear()
    self._chart.removeSeries(self._series)
    for (i, item) in enumerate(data):
      if not i % 77:
        print(item.real, item.imag)
      self._series.append(QPointF(item.real, item.imag))
    self._chart.addSeries(self._series)
    self._chart.axes()[0].setRange(tMin, tMax)
    self._chart.axes()[1].setRange(-2, 2)
    self._view.update()
    self.update()
