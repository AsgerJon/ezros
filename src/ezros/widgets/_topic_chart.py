"""TopicChart provides a widget for displaying real time updating data
received from a ros topic. The widget is based on the LiveChart widget,
complemented with a topic selection widget. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from PySide6.QtCore import Slot
from attribox import AttriBox
from icecream import ic
from PySide6.QtWidgets import QVBoxLayout, QHBoxLayout
from ezside.widgets import CanvasWidget, PushButton, BaseWidget
from ezside.widgets.charts import RealTimeView
from rospy import Subscriber, init_node
from vistutils.parse import maybe
from vistutils.waitaminute import typeMsg

from ezros.widgets import TopicComboBox, TopicSelector

ic.configureOutput(includeContext=True)


class TopicChart(CanvasWidget):
  """TopicChart provides a widget for displaying real time updating data
  received from a ros topic. The widget is based on the LiveChart widget,
  complemented with a topic selection widget. """

  baseLayout: QVBoxLayout
  realTimeView: RealTimeView
  sub: Subscriber

  topicSelector = AttriBox[TopicSelector]()

  __callback_functions__ = None

  @Slot()
  def _lockTopic(self, ) -> None:
    """Lock the topic selection."""

    rosTopic = self.topicSelector.rosTopic

    @rosTopic.subCreatorFactory('LMAO', )
    def callMeMaybe(data: Any) -> None:
      """Callback function for the anonymous subscriber that updates the
      chart with real time data."""
      self.realTimeView.append(data.data)

    if isinstance(callMeMaybe, Subscriber):
      self.sub = callMeMaybe
    else:
      e = typeMsg('callMeMaybe', callMeMaybe, Subscriber)
      raise TypeError(e)

  def _clearTopic(self) -> None:
    """Clear the topic selection."""
    self.sub.unregister()

  def initUi(self, ) -> None:
    """Initializes the user interface for the TopicChart."""
    #  Base Layout
    self.baseLayout = QVBoxLayout()
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(0)
    self.topicSelector.initUi()
    self.topicSelector.initSignalSlot()
    self.baseLayout.addWidget(self.topicSelector)
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
    self.topicSelector.topicSelected.connect(self._lockTopic)
    self.topicSelector.topicReset.connect(self._clearTopic)

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
