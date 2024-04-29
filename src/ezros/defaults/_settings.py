"""The defaults class provides a base class for all defaults classes in the
application. During development values are placed in the class body. Once
deployed values that should be configurable by the user are loaded from a
file. When the users make defaults changes, this file is updated."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Union

import ezside.core
from PySide6.QtCore import QRect, QPointF, QSizeF, QRectF
from PySide6.QtGui import QBrush
from PySide6.QtWidgets import QGraphicsRectItem
from ezside.core import SolidFill, SolidLine

PRect = Union[QRect, QRectF]
GRect = QGraphicsRectItem

from typing import Any

from PySide6.QtCharts import QChart, QValueAxis
from PySide6.QtGui import QFont, QColor, QPen


class Settings:
  """The defaults class provides a base class for all defaults classes in the
  application. During development values are placed in the class body. Once
  deployed values that should be configurable by the user are loaded from a
  file. When the users make defaults changes, this file is updated."""

  appFontFamily = 'Montserrat'

  publisherQueue = 10
  publisherInterval = 100

  #  Maximum number of points
  maxNumPoints = 256

  #  Time in milliseconds between updates of chart views
  chartUpdateInterval = 25

  #  Default timer type
  timerType = ezside.core.Precise

  #  Max age of chart items
  maxAge = 8

  #  Pump current plot ranges
  pumpCurrentMinView = -0.1
  pumpCurrentMaxView = 4
  pumpCurrentMaxSafe = 3.5
  pumpCurrentEpoch = 3
  pumpCurrentDataPoints = 128
  pumpCurrentDangerColor = {
    'red': 255,
    'green': 0,
    'blue': 0,
    'alpha': 63
  }

  pumpCurrentThemeName = 'BrownSand'

  pumpCurrentMarkerSize = 5

  pingFontSize = 16
  pingFontColor = QColor(0, 0, 63, 255)

  @classmethod
  def getPingFont(cls) -> QFont:
    """Getter-function for the ping font"""
    font = QFont(cls.appFontFamily)
    font.setPointSize(cls.pingFontSize)
    return font

  @classmethod
  def getPingPen(cls) -> QPen:
    """Getter-function for the ping pen"""
    pen = QPen()
    pen.setStyle(SolidLine)
    pen.setColor(cls.pingFontColor)
    pen.setWidth(1)
    return pen

  @classmethod
  def getPumpCurrentDangerColor(cls) -> QColor:
    """Getter-function for the danger color cast as instance of QColor"""
    return QColor(cls.pumpCurrentDangerColor['red'],
                  cls.pumpCurrentDangerColor['green'],
                  cls.pumpCurrentDangerColor['blue'],
                  cls.pumpCurrentDangerColor['alpha'])

  @classmethod
  def getPumpCurrentDangerBrush(cls) -> QBrush:
    """Getter-function for the danger brush"""
    dangerBrush = QBrush()
    dangerBrush.setColor(cls.getPumpCurrentDangerColor())
    dangerBrush.setStyle(SolidFill)
    return dangerBrush

  @classmethod
  def getPumpCurrentDangerRect(cls, viewRect: PRect) -> GRect:
    """Getter-function for the danger rectangle"""
    if isinstance(viewRect, QRect):
      viewRect = viewRect.toRectF()
    dangerPixelWidth = viewRect.width()
    dangerHeight = cls.pumpCurrentMaxView - cls.pumpCurrentMaxSafe
    viewHeight = cls.pumpCurrentMaxView - cls.pumpCurrentMinView
    dangerPixelHeight = viewRect.height() * dangerHeight / viewHeight
    dangerPixelLeft = viewRect.left()
    dangerPixelTop = viewRect.top()
    dangerPixelBottom = dangerPixelTop + dangerPixelHeight
    dangerPixelRight = viewRect.right()
    topLeft = QPointF(dangerPixelLeft, dangerPixelTop)
    size = QSizeF(dangerPixelWidth, dangerPixelHeight)
    rect = QRectF(topLeft, size)
    dangerRect = QGraphicsRectItem(rect)
    dangerRect.setBrush(cls.getPumpCurrentDangerBrush())
    dangerRect.setPen(ezside.core.emptyPen())
    return dangerRect

  @classmethod
  def getPumpCurrentXAxis(cls, ) -> QValueAxis:
    """Getter-function for the x-axis of the pump current plot"""
    xAxis = QValueAxis()
    xAxis.setRange(-cls.pumpCurrentEpoch, 0)
    xAxis.setTickCount(7)
    return xAxis

  @classmethod
  def getPumpCurrentYAxis(cls, ) -> QValueAxis:
    """Getter-function for the y-axis of the pump current plot"""
    yAxis = QValueAxis()
    yAxis.setRange(cls.pumpCurrentMinView, cls.pumpCurrentMaxView)
    yAxis.setTickCount(8)
    return yAxis

  @classmethod
  def getPumpCurrentTheme(cls) -> Any:
    """Getter-function for the pump current theme"""
    themeName = cls.pumpCurrentThemeName
    themes = {
      'BrownSand': QChart.ChartTheme.ChartThemeBrownSand,
      'BlueCerulean': QChart.ChartTheme.ChartThemeBlueCerulean,
      'Dark': QChart.ChartTheme.ChartThemeDark,
      'BlueNcs': QChart.ChartTheme.ChartThemeBlueNcs,
      'HighContrast': QChart.ChartTheme.ChartThemeHighContrast,
      'Light': QChart.ChartTheme.ChartThemeLight,
      'Qt': QChart.ChartTheme.ChartThemeQt,
      'BlueIcy': QChart.ChartTheme.ChartThemeBlueIcy,
    }
    for (key, val) in themes.items():
      if key.lower() == themeName.lower():
        return val
    return themes['BrownSand']
