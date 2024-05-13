"""LayoutWindow organizes the layouts used in the application"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QVBoxLayout
from ezside.app import BaseWindow
from ezside.core import AlignTop, AlignLeft
from ezside.widgets import BaseWidget
from icecream import ic

from ezros.widgets import TopicSelection, TopicChart

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  debug1: QAction
  debug2: QAction
  debug3: QAction
  debug4: QAction
  debug5: QAction
  debug6: QAction
  debug7: QAction
  debug8: QAction
  debug9: QAction
  baseLayout: QVBoxLayout
  baseWidget: BaseWidget
  topicChart: TopicChart

  @abstractmethod
  def initSignalSlot(self) -> None:
    """Initialize the signal-slot connections."""

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMinimumSize(480, 320)
    #  Layout and base widget
    self.baseLayout = QVBoxLayout()
    self.baseLayout.setAlignment(AlignTop | AlignLeft)
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(0)
    self.baseWidget = BaseWidget()
    #  TopicSelection
    self.topicChart = TopicChart()
    self.topicChart.initUi()
    self.baseLayout.addWidget(self.topicChart)
    #  Setting layout and central widget
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
