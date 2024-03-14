"""TabField wraps QTabWidget in a closure."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from time import sleep
from typing import Any
from warnings import warn

from PySide6.QtCore import Qt, Slot, QSize
from PySide6.QtGui import QPaintEvent
from PySide6.QtWidgets import QTabWidget
from icecream import ic
from vistside.core import parseParent
from vistside.widgets import LabelField, BaseWidget, BaseLayoutField
from vistutils.fields import unParseArgs, Wait

from ezros.gui import DynPlotField, DynPlot

TopLeft = Qt.Corner.TopLeftCorner
TopRight = Qt.Corner.TopRightCorner


class TabWidget(QTabWidget):
  """TabWidget is a wrapper around QTabWidget. It is used to create a tabbed
  layout in the GUI.
  """
  cornerBanner = LabelField('Yolo')
  plotBase = Wait(BaseWidget)
  plotLayout = BaseLayoutField(layout='vertical')
  plotBanner = LabelField('Spray Tool')
  plotWidget = DynPlotField(256)
  controlBase = Wait(BaseWidget)
  controlLayout = BaseLayoutField(layout='vertical')
  controlBanner = LabelField('Pump Control')
  controlWidget = LabelField('Here be buttons')

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the TabWidget."""
    parent = parseParent(*args)
    QTabWidget.__init__(self, parent)

  def initUI(self, ) -> None:
    """Initializes the user interface."""
    self.plotLayout.addWidget(self.plotBanner)
    self.plotWidget.setMinimumSize(QSize(400, 200))
    self.plotWidget.initUI()
    self.plotLayout.addWidget(self.plotWidget)
    self.plotBase.setLayout(self.plotLayout)
    self.addTab(self.plotBase, 'Spray Tool')
    # self.controlLayout.addWidget(self.controlBanner)
    # self.controlLayout.addWidget(self.controlWidget)
    # self.controlBase.setLayout(self.controlLayout)
    # self.addTab(self.controlBase, 'Pump Control')
    # self.setCornerWidget(self.cornerBanner, TopRight)

  @Slot(float)
  def callback(self, value: float) -> None:
    """Appends new data value. """
    self.plotWidget.callback(value)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    try:
      self.plotWidget.updateChart()
    except Exception as e:
      warn(str(e))
    QTabWidget.paintEvent(self, event)

  @classmethod
  def getDefault(cls, *args, **kwargs) -> TabWidget:
    """Returns the default value for the field."""
    parent = parseParent(*args)
    return cls(parent).apply((args, kwargs))

  def apply(self, value: Any) -> TabWidget:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class TabField(Wait):
  """TabField is a field for a TabWidget."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the TabField."""
    Wait.__init__(self, TabWidget, *args, **kwargs)
