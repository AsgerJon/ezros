"""TopicChart provides a widget for displaying real time updating data
received from a ros topic. The widget is based on the LiveChart widget,
complemented with a topic selection widget. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from PySide6.QtWidgets import QVBoxLayout
from ezside.widgets import CanvasWidget, LiveChart, PushButton, Label

from ezros.rosutils import RosTopic
from ezros.widgets import TopicComboBox


class TopicChart(CanvasWidget):
  """TopicChart provides a widget for displaying real time updating data
  received from a ros topic. The widget is based on the LiveChart widget,
  complemented with a topic selection widget. """

  baseLayout: QVBoxLayout
  topicComboBox: TopicComboBox
  selectButton: PushButton
  clearButton: PushButton
  liveData: LiveChart
  footerLabel: Label

  def callbackFactory(self, topic: str | RosTopic) -> Callable:
    """The callbackFactory method creates a callback function for the
    topic."""

    def callMeMaybe(data: Any) -> None:
      """The callMeMaybe method is the callback function for the topic."""
      self.masterCallback(data.data)

    return callMeMaybe

  def masterCallback(self, data: float) -> None:
    """The masterCallback method is the callback function for the topic."""

  def initUi(self, ) -> None:
    """The initUi method initializes the user interface for the widget."""
    self.baseLayout = QVBoxLayout(self)
    self.baseLayout.setSpacing(0)
    self.baseLayout.setContentsMargins(0, 0, 0, 0)

    self.topicComboBox = TopicComboBox()
    self.baseLayout.addWidget(self.topicComboBox)

    self.selectButton = PushButton('Select', )
    self.selectButton.initUi()
    self.selectButton.initSignalSlot()
    self.baseLayout.addWidget(self.selectButton)

    self.liveData = LiveChart(self)
    self.liveData.initUi()
    self.baseLayout.addWidget(self.liveData)

    self.setLayout(self.baseLayout)
