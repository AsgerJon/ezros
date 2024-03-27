"""DynWidget provides a widget view of the live updating chart."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QXYSeries, QChart, QChartView, QScatterSeries
from PySide6.QtCore import Slot
from PySide6.QtGui import QPainter
from attribox import AttriBox
from ezside.widgets import BaseWidget
from icecream import ic
from msgs.msg import Float32Stamped
from vistutils.parse import maybe

from ezros.defaults import Settings
from ezros.rosutils import RollingArray
from ezros.widgets import Vertical

ic.configureOutput(includeContext=True)


class DynWidget(BaseWidget):
  """DynWidget provides a widget view of the live updating chart."""

  baseLayout = AttriBox[Vertical]()

  def __init__(self, numPoints: int = None) -> None:
    BaseWidget.__init__(self)
    self._numPoints = maybe(numPoints, Settings.numPoints)
    self._rollingArray = RollingArray(numPoints)
    self._series = QScatterSeries()
    self._series.appendNp(*self._rollingArray.rightNow())
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

  @Slot(Float32Stamped)
  def append(self, data: Float32Stamped) -> None:
    """Append data to the rolling array."""
    self._rollingArray.append(data)

  @Slot()
  def refresh(self) -> None:
    """Refresh the view."""
    self._series.clear()
    self._series.appendNp(*self._rollingArray.rightNow())
    self._view.repaint()
    self.update()
