"""TopicInfo shows the name of a topic and the name of the topic type.
Please note that this type does not provide any validation of the topic."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QColor, QBrush
from PySide6.QtWidgets import QVBoxLayout
from ezside.core import parseBrush, \
  SolidFill, \
  parsePen, \
  SolidLine, \
  Normal, \
  parseFont
from ezside.widgets import BaseWidget, Label
from icecream import ic

ic.configureOutput(includeContext=True)


class _TopicLabel(Label):
  """_TopicLabel provides a label for the topic info widget."""

  @classmethod
  def styleTypes(cls) -> dict[str, Any]:
    """The staticTypes method provides the static types for the widget."""
    LabelStyleTypes = Label.styleTypes()
    topicStyleTypes = {}
    return {**LabelStyleTypes, **topicStyleTypes}

  @classmethod
  def staticStyles(cls, ) -> dict[str, Any]:
    """The staticStyles method provides the static styles for the widget."""
    LabelStyles = Label.styleTypes()
    topicStyles = {
      'backgroundBrush': parseBrush(QColor(0, 0, 31, 255, ), SolidFill, ),
      'textPen'        : parsePen(QColor(144, 255, 0, 255), 1, SolidLine, ),
      'font'           : parseFont('Consolas', 16, Normal)
    }
    return {**LabelStyles, **topicStyles}

  def dynStyles(self, ) -> dict[str, Any]:
    """The dynStyles method provides the dynamic styles for the widget."""
    return {}


class TopicInfo(_TopicLabel):
  """TopicInfo shows the name of a topic and the name of the topic type.
  Please note that this type does not provide any validation of the topic."""

  baseLayout: QVBoxLayout
  topicName: Label
  topicType: Label
  topicValue: Label
  __topic_list__ = None
  __topic_names__ = []
  __topic_dict__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the TopicInfo widget."""
    data = kwargs | dict(id='topic')
    _TopicLabel.__init__(self, *args, **data)

  def initUi(self) -> None:
    """Initializes the user interface of the widget."""
    self.baseLayout = QVBoxLayout(self)
    self.topicName = _TopicLabel()
    self.topicName.initUi()
    self.topicType = _TopicLabel()
    self.topicType.initUi()
    self.topicValue = _TopicLabel()
    self.topicValue.initUi()
    self.baseLayout.addWidget(self.topicName)
    self.baseLayout.addWidget(self.topicType)
    self.baseLayout.addWidget(self.topicValue)
    self.setLayout(self.baseLayout)

  def initSignalSlot(self) -> None:
    """Initializes the signal-slot connections of the widget."""

  def _setTopicName(self, topicName: str) -> None:
    """Sets the topic name type."""
    text = 'Topic Name: %s' % topicName
    self.topicName.text(text)
    self.topicName.update()

  def _setTopicType(self, topicType: str) -> None:
    """Sets the topic name"""
    text = 'Topic Type: %s' % topicType
    self.topicType.text(text)
    self.topicType.update()
