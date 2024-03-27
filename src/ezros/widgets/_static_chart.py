"""StaticChart leverages QChart to visualize data."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from PySide6.QtCharts import QChartView, QChart, QLineSeries
from PySide6.QtGui import QPainter, QColor, QFont
from PySide6.QtWidgets import QVBoxLayout
from attribox import AttriBox
from ezside.widgets import BaseWidget
import numpy as np


class _StaticChart(QChartView):
  """StaticChart leverages QChart to visualize data."""

  __data_chart__ = None
  __data_series__ = None

  def __init__(self, func: Callable, *args) -> None:
    QChartView.__init__(self, )
    x0, x1, N = None, None, None
    for arg in args:
      if isinstance(arg, (int, float)):
        if x0 is None:
          x0 = arg
        elif x1 is None:
          x1 = arg
        elif N is None:
          N = arg
        if x0 is not None and x1 is not None and N is not None:
          break
    else:
      if x1 is None:
        e = """Low and high limits must be provided!"""
        raise ValueError(e)
      if N is None:
        N = 128
    _x = np.linspace(x0, x1, N).astype(np.float32)
    _y = func(_x).astype(np.float32)
    titleFont = QFont()
    titleFont.setPointSize(18)
    titleFont.setFamily('Montserrat')
    self.__data_series__ = QLineSeries()
    self.__data_series__.appendNp(_x, _y)
    self.__data_series__.setName("Test Data")
    self.__data_series__.setColor(QColor(0, 0, 255))
    self.__data_chart__ = QChart()
    self.__data_chart__.addSeries(self.__data_series__)
    self.__data_chart__.createDefaultAxes()
    self.__data_chart__.setTitle("Simple scatterchart example")
    self.__data_chart__.setDropShadowEnabled(True)
    self.__data_chart__.setTheme(QChart.ChartTheme.ChartThemeBrownSand)
    self.setChart(self.__data_chart__)
    self.chart().setTitleFont(titleFont)
    self.repaint()


class StaticChart(BaseWidget):
  """StaticChart leverages QChart to visualize data."""

  baseLayout = AttriBox[QVBoxLayout]()

  def __init__(self, func: Callable, *args) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, )
    self.inner = _StaticChart(func, *args)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.inner)
    self.setLayout(self.baseLayout)
