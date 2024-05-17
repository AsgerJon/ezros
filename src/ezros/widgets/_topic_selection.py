"""TopicSelection provides topic selection functionality. A dropdown menu
lists all available topics, a lineedit enables filtering, and a
confirmation button locks the topic. Once locked, the topic can be changed
with a clear button."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QHBoxLayout
from attribox import AttriBox
from ezside.core import parseBrush, SolidFill, parsePen, SolidLine
from ezside.widgets import Label, PushButton, CanvasWidget

from ezros.rosutils import RosTopic
from ezros.widgets import TopicComboBox


class TopicSelection(CanvasWidget):
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

  def initUi(self, ) -> None:
    """Initializes the user interface for the TopicSelection."""
    #  Base Layout
    self.baseLayout = QHBoxLayout()
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(4)
    #  TopicComboBox
    self.baseLayout.addWidget(self.topicComboBox)
    #  ClearButton
    self.baseLayout.addWidget(self.clearButton)
    #  SelectButton
    self.baseLayout.addWidget(self.selectButton)
    self.setLayout(self.baseLayout)

  def initSignalSlot(self) -> None:
    """Initializes the signal-slot connections."""
    self.clearButton.clicked.connect(self.resetTopic)
    self.topicComboBox.currentItemChanged.connect(self.viewTopic)
    self.selectButton.clicked.connect(self.selectTopic)

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

  @Slot(RosTopic)
  def selectTopic(self, topic: RosTopic) -> None:
    """Slot that selects the topic. """
    self.clearButton.setEnabled(True)
    self.selectButton.setEnabled(False)
    name = topic.topicName
    self.topicComboBox.lineEdit().setPrefix('Selected: %s  | ' % name)
    self.topicSelected.emit(self.topicComboBox.currentItem())
