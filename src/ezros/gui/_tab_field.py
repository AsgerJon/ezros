"""TabField wraps QTabWidget in a closure."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any
from warnings import warn

from PySide6.QtCore import Qt, Slot, QSize
from PySide6.QtGui import QPaintEvent
from PySide6.QtWidgets import QTabWidget
from vistside.core import parseParent
from vistside.widgets import LabelField, BaseWidget, BaseLayoutField
from vistutils.fields import unParseArgs, Wait, FieldBox

from ezros.gui import DynPlot

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
  # plotWidget = FieldBox[DynPlot](256)
  controlBase = Wait(BaseWidget)
  controlLayout = BaseLayoutField(layout='vertical')
  controlBanner = LabelField('Pump Control')
  controlWidget = LabelField('Here be buttons')

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the TabWidget."""
    parent = parseParent(*args)
    QTabWidget.__init__(self, parent)
    self.plotWidget = DynPlot(256, self)

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
