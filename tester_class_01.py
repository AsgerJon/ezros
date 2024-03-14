"""Tester class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from math import pi, sin
from typing import Any, Self

from PySide6.QtCharts import QChartView, QLineSeries, QChart
from PySide6.QtCore import QPointF, Signal, QRectF, QEvent
from PySide6.QtGui import QPainter, QPaintEvent, QMouseEvent
from PySide6.QtWidgets import QGraphicsItem, QGraphicsRectItem, QWidget, \
  QGraphicsView, QMainWindow
from icecream import ic
from vistside.core import parsePen, Black
from vistside.widgets import BaseWidget, BaseLayoutField, LabelWidget, \
  LabelField
from vistutils.fields import Wait, unParseArgs

ic.configureOutput(includeContext=True)


class Chart(QChart):
  """Wrapper class providing closure and descriptor."""

  @staticmethod
  def createSeries() -> QLineSeries:
    """Creates a series."""
    series = QLineSeries()
    for i in range(128):
      t = 2 * pi / 127 * i
      x = sin(t)
      series.append(t, x)
    return series

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the Chart."""
    QChart.__init__(self)
    self.legend().setBorderColor(Black)
    self.setBackgroundRoundness(8)
    self.setTheme(QChart.ChartTheme.ChartThemeBrownSand)
    self.setAnimationOptions(QChart.AnimationOption.NoAnimation)
    self.addSeries(self.createSeries())
    self.createDefaultAxes()

  def getRect(self, t0: float, t1: float, x0: float, x1: float) -> QRectF:
    """Returns a rectangle."""
    topLeft = self.mapToPosition(QPointF(t0, x0))
    bottomRight = self.mapToPosition(QPointF(t1, x1))
    rect = QRectF(topLeft, bottomRight)
    ic(rect.left(), rect.top(), rect.right(), rect.bottom())
    return QGraphicsRectItem(rect)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ChartField(Wait):
  """The ChartField class provides a descriptor for instances of Chart."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ChartField."""
    Wait.__init__(self, Chart, *args, **kwargs)


class View(QChartView):
  """Wrapper class providing closure and descriptor."""

  __bottom_rect__ = None

  def getMain(self) -> QWidget:
    """Returns the main window."""

    def getParent(widget: QWidget) -> QWidget:
      """Returns the parent widget."""
      if widget.parent() is None:
        return widget
      return getParent(widget.parent())

    return getParent(self)

  chart = ChartField()
  mouse = Signal(float, float)

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the View."""
    QChartView.__init__(self, *args, **kwargs)
    self.setChart(self.chart)
    self.setRenderHint(QPainter.RenderHint.Antialiasing)
    self.setMouseTracking(True)
    self.setViewportUpdateMode(
      QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
    self.bottomRect = None

  def createBottomRect(self) -> None:
    """Creator function for the bottom rect"""
    self.__bottom_rect__ = self.chart.getRect(0, 2 * pi, -1, -0.8)

  def getBottomRect(self, **kwargs) -> QGraphicsRectItem:
    """Returns the bottom rectangle."""
    if self.__bottom_rect__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createBottomRect()
      return self.getBottomRect(_recursion=True)
    return self.__bottom_rect__

  def addTop(self) -> None:
    """Adds a top item."""

  def mousePressEvent(self, event: QMouseEvent) -> None:
    """Handles the mouse press event."""
    rect = self.chart.getRect(0, 2 * pi, -1, -0.8)
    self.scene().addItem(rect)
    self.scene().update(rect.rect())

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Handles the mouse release event."""
    self.parent().update()
    self.getMain().update()

  def mouseMoveEvent(self, event: QMouseEvent) -> None:
    """Handles the mouse move event."""
    self.mouse.emit(event.x(), event.y())

  def showEvent(self, event: QEvent) -> None:
    """Handles the show event."""
    self.scene().addItem(self.getBottomRect())
    QChartView.showEvent(self, event)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ViewField(Wait):
  """The ViewField class provides a descriptor for instances of View."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ViewField."""
    Wait.__init__(self, View, *args, **kwargs)


class TestWidget(BaseWidget):
  """The TestWidget class is a widget for testing purposes."""

  baseLayout = BaseLayoutField(layout='vertical')
  banner = LabelField('Yolo')
  viewField = ViewField()
  mouse = Signal(float, float)

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestWidget."""
    BaseWidget.__init__(self, *args, **kwargs)
    self._view = None

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.viewField)
    self.setLayout(self.baseLayout)
    self.viewField.mouse.connect(self.mouse)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> TestWidget:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> TestWidget:
    """Applies the arguments contained in value to the widget."""
    return self


class TestField(Wait):
  """The TestField class is a field for testing purposes."""

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestField."""
    Wait.__init__(self, TestWidget, *args, **kwargs)
