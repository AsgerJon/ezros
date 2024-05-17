"""TopicSelection provides topic selection functionality. A dropdown menu
lists all available topics, a lineedit enables filtering, and a
confirmation button locks the topic. Once locked, the topic can be changed
with a clear button."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QHBoxLayout
from attribox import AttriBox
from ezside.core import Prefer, Expand, Tight
from ezside.widgets import PushButton, CanvasWidget
from icecream import ic
from vistutils.fields import EmptyField

from ezros.rosutils import RosTopic
from ezros.widgets import TopicComboBox

ic.configureOutput(includeContext=True)


class TopicSelector(CanvasWidget):
  """TopicSelection provides topic selection functionality. A dropdown menu
  lists all available topics, a lineedit enables filtering, and a
  confirmation button locks the topic. Once locked, the topic can be changed
  with a clear button."""

  baseLayout = AttriBox[QHBoxLayout]()
  clearButton = AttriBox[PushButton]('RESET', )
  topicComboBox = AttriBox[TopicComboBox]()
  selectButton = AttriBox[PushButton]('Select')

  topicReset = Signal()
  topicViewed = Signal(RosTopic)
  topicSelected = Signal(RosTopic)

  topicName = EmptyField()
  topicType = EmptyField()
  rosTopic = EmptyField()

  @topicName.GET
  def _getTopicName(self) -> str:
    """Getter-function for the topic name."""
    return self.topicComboBox.currentItem().topicName

  @topicType.GET
  def _getTopicType(self) -> str:
    """Getter-function for the topic type."""
    return self.topicComboBox.currentItem().topicType

  @rosTopic.GET
  def _getRosTopic(self) -> RosTopic:
    """Getter-function for the ROS topic."""
    return self.topicComboBox.currentItem()

  def initUi(self, ) -> None:
    """Initializes the user interface for the TopicSelection."""
    #  Base Layout
    self.baseLayout = QHBoxLayout()
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(4)
    #  ClearButton
    self.clearButton.setEnabled(False)
    self.clearButton.initUi()
    self.baseLayout.addWidget(self.clearButton)
    #  TopicComboBox
    self.topicComboBox.setSizePolicy(Expand, Tight)
    self.baseLayout.addWidget(self.topicComboBox)
    #  SelectButton
    self.selectButton.setEnabled(True)
    self.selectButton.initUi()
    self.baseLayout.addWidget(self.selectButton)
    self.setLayout(self.baseLayout)

  def initSignalSlot(self) -> None:
    """Initializes the signal-slot connections."""
    self.clearButton.initSignalSlot()
    self.selectButton.initSignalSlot()
    self.clearButton.singleLeft.connect(self.resetTopic)
    self.topicComboBox.currentIndexChanged.connect(self._topicViewChange)
    self.selectButton.singleLeft.connect(self.selectTopic)

  def _topicViewChange(self, index: int) -> None:
    """Handles the topic view changed signal. """
    ic(self.topicComboBox[index])

  @Slot()
  def resetTopic(self) -> None:
    """Slot that resets the topic selection. """
    self.clearButton.setEnabled(False)
    self.clearButton.update()
    self.topicReset.emit()

  @Slot(RosTopic)
  def viewTopic(self, topic: RosTopic) -> None:
    """Slot that views the selected topic. """
    self.topicViewed.emit(topic)

  @Slot()
  def selectTopic(self, ) -> None:
    """Slot that selects the topic. """
    ic('lmao')
    self.clearButton.setEnabled(True)
    self.selectButton.setEnabled(False)
    topic = self.topicComboBox.currentItem()
    self.topicSelected.emit(topic)
    self.update()
