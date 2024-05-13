"""TopicChart provides a widget for displaying real time updating data
received from a ros topic. The widget is based on the LiveChart widget,
complemented with a topic selection widget. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from PySide6.QtCore import Slot
from icecream import ic
from PySide6.QtWidgets import QVBoxLayout, QHBoxLayout
from ezside.widgets import CanvasWidget, PushButton, BaseWidget
from ezside.widgets.charts import RealTimeView
from rospy import Subscriber
from vistutils.parse import maybe

from ezros.widgets import TopicComboBox

ic.configureOutput(includeContext=True)


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
  sub: Subscriber

  __callback_functions__ = None

  @Slot()
  def _lockTopic(self, ) -> None:
    """Lock the topic selection."""
    subCreator = self.topicComboBox.currentItem().subCreatorFactory()

    @subCreator
    def callMeMaybe(data: Any) -> None:
      """Callback function for the anonymous subscriber that updates the
      chart with real time data."""
      print(data.data)
      self.realTimeView.append(data.data)

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
    self.realTimeView.__inner_chart__.legend().setVisible(False)
    self.realTimeView.__inner_chart__.axes()[0].setRange(-10, 10)
    self.realTimeView.initUi()
    self.baseLayout.addWidget(self.realTimeView)
    #  Setting layout
    self.setLayout(self.baseLayout)

  def initSignalSlot(self, ) -> None:
    """Initializes the signal slot connections for the TopicChart."""
    self.selectButton.singleClick.connect(self._lockTopic)

  def _getCallbackFunctions(self) -> list[Callable]:
    """Get the callback functions for the TopicChart."""
    return maybe(self.__callback_functions__, [])

  def _appendCallback(self, callback: Callable) -> None:
    """Append a callback function to the TopicChart."""
    self.__callback_functions__ = [*self._getCallbackFunctions(), callback]

  def _clearCallbacks(self) -> None:
    """Clear the callback functions for the TopicChart."""
    self.__callback_functions__ = []

  def generalCallback(self, *args) -> None:
    """General callback for the TopicChart."""
    for callMeMaybe in self._getCallbackFunctions():
      callMeMaybe(*args)

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
