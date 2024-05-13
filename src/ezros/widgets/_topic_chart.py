"""TopicChart provides a widget for displaying real time updating data
received from a ros topic. The widget is based on the LiveChart widget,
complemented with a topic selection widget. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtWidgets import QVBoxLayout, QHBoxLayout
from ezside.widgets import CanvasWidget, PushButton, BaseWidget
from ezros.widgets import TopicComboBox
from ezside.widgets.charts import RealTimeView


class TopicChart(CanvasWidget):
  """TopicChart provides a widget for displaying real time updating data
  received from a ros topic. The widget is based on the LiveChart widget,
  complemented with a topic selection widget. """

  baseLayout: QVBoxLayout
  headerLayout: QHBoxLayout
  headerWidget: BaseWidget
  clearButton: PushButton
  topicComboBox: TopicComboBox
  selectButton: PushButton
  realTimeView: RealTimeView

  def initUi(self, ) -> None:
    """Initializes the user interface for the TopicChart."""
    #  Base Layout
    self.baseLayout = QVBoxLayout()
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(0)
    #  Header Layout
    self.headerLayout = QHBoxLayout()
    self.headerLayout.setContentsMargins(0, 0, 0, 0, )
    self.headerLayout.setSpacing(0)
    #  Clear Button
    self.clearButton = PushButton('Clear')
    self.clearButton.initUi()
    self.headerLayout.addWidget(self.clearButton)
    #  Topic ComboBox
    self.topicComboBox = TopicComboBox()
    self.headerLayout.addWidget(self.topicComboBox)
    #  Select Button
    self.selectButton = PushButton('Select')
    self.selectButton.initUi()
    self.headerLayout.addWidget(self.selectButton)
    #  Header Layout Widget
    self.headerWidget = BaseWidget()
    self.headerWidget.setLayout(self.headerLayout)
    self.baseLayout.addWidget(self.headerWidget)
    #  Real Time View
    self.realTimeView = RealTimeView()
    self.realTimeView.initUi()
    self.baseLayout.addWidget(self.realTimeView)
    #  Setting layout
    self.setLayout(self.baseLayout)

  def initSignalSlot(self, ) -> None:
    """Initializes the signal slot connections for the TopicChart."""

  @classmethod
  def styleTypes(cls) -> dict[str, type]:
    """The styleTypes method provides the type expected at each name."""
    canvasWidgetStyleTypes = CanvasWidget.styleTypes()
    topicChartStyleTypes = {}
    return {**canvasWidgetStyleTypes, **topicChartStyleTypes}

  @classmethod
  def staticStyles(cls) -> dict[str, Any]:
    """Returns the static styles for the TopicChart."""
    canvasWidgetStyles = CanvasWidget.staticStyles()
    topicChartStyles = {}
    return {**canvasWidgetStyles, **topicChartStyles}

  def dynStyles(self, ) -> dict[str, Any]:
    """Implementation of dynamic fields"""
    return {}
