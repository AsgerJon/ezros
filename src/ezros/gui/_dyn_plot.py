"""DynPlot provides a dynamic widget visualizing data in real-time."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

import numpy as np
from PySide6.QtCharts import QChart, QScatterSeries, QChartView
from PySide6.QtCharts import QXYSeries
from PySide6.QtCore import Slot, QSize, QPointF, QRectF, QEvent, QSizeF
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget, QGraphicsRectItem
from icecream import ic
from vistside.core import parseParent, Black, parseBrush, \
  SolidFill
from vistside.core import emptyPen
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FloatField, FieldBox
from vistutils.waitaminute import typeMsg

from ezros.rosutils import Array

QGRect = QGraphicsRectItem
PointColor = QXYSeries.PointConfiguration.Color


class Chart(QChart):
  """Wrapper class providing closure and descriptor."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the Chart."""
    QChart.__init__(self)
    self.legend().setBorderColor(Black)
    self.setBackgroundRoundness(96)
    self.setTheme(QChart.ChartTheme.ChartThemeBrownSand)
    self.setAnimationOptions(QChart.AnimationOption.NoAnimation)


class View(QChartView):
  """Wrapper class providing closure and descriptor."""

  __explicit_chart__ = None
  array = FieldBox[Array]()

  minView = FloatField(-1.5)
  minWarn = FloatField(-1.0)
  maxWarn = FloatField(1.0)
  maxView = FloatField(1.5)
  epochTime = FloatField(10)

  @Slot()
  def appendValue(self, value: float) -> None:
    """Appends a value to the array."""
    self.array.append(value)

  @Slot()
  def setAir(self, ) -> None:
    """Sets the air."""
    self.minWarn = -0.5
    self.maxWarn = 0.5

  @Slot()
  def setWater(self, ) -> None:
    """Sets the water."""
    self.minWarn = -0.75
    self.maxWarn = 0.75

  @Slot()
  def setPaint(self, ) -> None:
    """Sets the paint."""
    self.minWarn = -1.0
    self.maxWarn = 1.0

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the View."""
    parent = parseParent(*args)
    QChartView.__init__(self, parent)
    self.setRubberBand(QChartView.RubberBand.NoRubberBand)
    self.setRenderHint(QPainter.Antialiasing)

  def setChart(self, chart: QChart) -> None:
    """Sets the chart."""

  def showEvent(self, event: QEvent) -> None:
    """Handles the show event."""
    return QChartView.showEvent(self, event)
    rects = self.getRect()
    for rect in rects:
      self.scene().addItem(rect)
    self.chart()
    QChartView.showEvent(self, event)

  def mapY2Pixel(self, y: float) -> float:
    """Maps the y-coordinate to a pixel."""
    return self.chart().mapToPosition(QPointF(0, y)).y()

  def mapX2Pixel(self, x: float) -> float:
    """Maps the x-coordinate to a pixel."""
    return self.chart().mapToPosition(QPointF(x, 0)).x()

  def getRect(self, *args, **kwargs) -> tuple[QGRect, QGRect]:
    """Returns the rectangle."""
    plotArea = self.chart().plotArea()
    width = plotArea.width()
    left, top, bottom = plotArea.left(), plotArea.top(), plotArea.bottom()
    brush = parseBrush(QColor(255, 0, 0, 31), SolidFill)
    pen = emptyPen()
    data = self.chart().series()[0]
    if not isinstance(data, QXYSeries):
      e = typeMsg('data', data, QXYSeries)
      raise TypeError(e)
    lowWarnTop = self.mapY2Pixel(self.minWarn)
    highWarnBottom = self.mapY2Pixel(self.maxWarn)
    lowWarnTopLeft = QPointF(left, lowWarnTop)
    lowWarnHeight = bottom - lowWarnTop
    lowWarnSize = QSizeF(width, lowWarnHeight)
    lowWarn = QRectF(lowWarnTopLeft, lowWarnSize)
    minGraphicRect = QGraphicsRectItem(lowWarn)
    highWarnTopLeft = QPointF(left, top)
    highWarnHeight = highWarnBottom - top
    highWarnSize = QSizeF(width, highWarnHeight)
    highWarn = QRectF(highWarnTopLeft, highWarnSize)
    maxGraphicRect = QGraphicsRectItem(highWarn)
    minGraphicRect.setBrush(brush)
    minGraphicRect.setPen(pen)
    maxGraphicRect.setBrush(brush)
    maxGraphicRect.setPen(pen)
    return minGraphicRect, maxGraphicRect


class DynPlot(QWidget):
  """The DynPlot class provides a dynamic widget visualizing data in
  real-time."""

  __fallback_num_points__ = 128
  baseLayout = BaseLayoutField(layout='vertical')

  dataChart = FieldBox[Chart]()
  array = FieldBox[Array]()
  data = FieldBox[QScatterSeries]()
  view = FieldBox[View]()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new DynPlot."""
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
    self.setMinimumSize(QSize(480, 320))
    self._zeroTime = time.time()

  def updateChart(self) -> None:
    """Updates the chart."""
    P = self.array.snap()
    if not round(time.time()) % 10:
      ic(P)
    X, Y = P.real.astype(np.float32), P.imag.astype(np.float32)
    self.data.clear()
    self.data.appendNp(X, Y)

  def setupView(self) -> None:
    """Sets up the view."""
    ic('self.setupView()')
    self.dataChart.addSeries(self.data)
    self.dataChart.createDefaultAxes()
    self.dataChart.axes()[1].setRange(-10, 10)
    self.view.setChart(self.dataChart)
    self.view.setAir()

  def initUI(self, ) -> None:
    """Initializes the user interface."""
    ic('self.initUI()')
    self.setupView()
    self.baseLayout.addWidget(self.view)
    self.setLayout(self.baseLayout)

  @Slot(float)
  def callback(self, value: float) -> None:
    """Appends the value to the data."""
    self.array.append(value)

  def showEvent(self, event) -> None:
    """Shows the widget."""
    self.setupView()
    self.view.showEvent(event)
    BaseWidget.showEvent(self, event)
