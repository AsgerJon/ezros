"""DynChart provides a dynamic chart widget for plotting data in
real-time. The widget refreshes its graphical content at a fixed and
independent rate separate from when new data is added to the chart."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QChartView
from PySide6.QtCore import Slot
from PySide6.QtWidgets import QVBoxLayout, QGridLayout, QHBoxLayout
from attribox import AttriBox
from ezside.core import Precise
from ezside.widgets import Timer, BaseWidget, DataView
from icecream import ic
from vistutils.waitaminute import typeMsg

from ezros.widgets import SpinBox

ic.configureOutput(includeContext=True)


class DynChart(BaseWidget):
  """DynChart provides a dynamic chart widget for plotting data in
  real-time. The widget refreshes its graphical content at a fixed and
  independent rate separate from when new data is added to the chart."""

  baseLayout = AttriBox[QVBoxLayout]()
  dataView = AttriBox[DataView]()
  timer = AttriBox[Timer](10, Precise, singleShot=False)
  controlsLayout = AttriBox[QHBoxLayout]()
  controlsWidget = AttriBox[BaseWidget]()
  minH = AttriBox[SpinBox]('Left', -10, 0, 10)
  maxH = AttriBox[SpinBox]('Right', 0, 5, 10)
  minV = AttriBox[SpinBox]('Bottom', -10, 0, 10)
  maxV = AttriBox[SpinBox]('Top', 0, 5, 10)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.dataView.initUi()
    self.baseLayout.addWidget(self.dataView)
    self.minH.initUi()
    self.controlsLayout.addWidget(self.minH)
    self.maxH.initUi()
    self.controlsLayout.addWidget(self.maxH)
    self.minV.initUi()
    self.controlsLayout.addWidget(self.minV)
    self.maxV.initUi()
    self.controlsLayout.addWidget(self.maxV)
    self.controlsWidget.setLayout(self.controlsLayout)
    self.updateRanges()
    self.baseLayout.addWidget(self.controlsWidget)
    self.setLayout(self.baseLayout)
    BaseWidget.initUi(self)

  def connectActions(self) -> None:
    """Connect actions to slots."""
    self.minH.update.connect(self.updateRanges)
    self.maxH.update.connect(self.updateRanges)
    self.minV.update.connect(self.updateRanges)
    self.maxV.update.connect(self.updateRanges)
    self.timer.timeout.connect(self.dataView.refresh)
    self.timer.start()

  @Slot(float)
  def append(self, value: float) -> None:
    """Append a value to the chart."""
    if isinstance(value, float):
      return self.dataView.append(value)
    if isinstance(value, int, ):
      return self.dataView.append(float(value))
    e = typeMsg('value', value, float)
    raise TypeError(e)

  def updateRanges(self) -> None:
    """Update the ranges of the chart."""
    self.setVerticalRange(self.minV.inner.value(), self.maxV.inner.value())
    self.setHorizontalRange(self.minH.inner.value(), self.maxH.inner.value())
    self.update()
    self.dataView.refresh()

  def setVerticalRange(self, min_: float, max_: float) -> None:
    """Set the vertical range of the chart."""
    ic(min_, max_)
    self.dataView.innerChart.axes()[1].setRange(min_, max_)
    self.update()
    self.dataView.refresh()
    self.update()

  def setHorizontalRange(self, min_: float, max_: float) -> None:
    """Set the horizontal range of the chart."""

    self.dataView.innerChart.axes()[0].setRange(min_, max_)
    self.update()
    self.dataView.refresh()
    self.update()
