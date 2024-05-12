"""TopicSelection provides topic selection functionality. A dropdown menu
lists all available topics, a lineedit enables filtering, and a
confirmation button locks the topic. Once locked, the topic can be changed
with a clear button."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import QMargins, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QGridLayout, QLineEdit
from ezside.core import Prefer, parseBrush, SolidFill, parsePen, SolidLine
from ezside.widgets import BaseWidget, Label, PushButton
from icecream import ic
from rospy import Subscriber
from vistutils.fields import EmptyField
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.widgets import TopicComboBox


class _Button(PushButton):
  """_Button provides a button for the topic selection widget."""

  singleClick = Signal()

  def dynStyles(self, ) -> dict[str, Any]:
    """The dynStyles method provides the dynamic styles for the widget."""
    pushButtonDynStyles = PushButton.dynStyles(self)
    if self.__is_enabled__:
      return pushButtonDynStyles
    if self.__is_hovered__:
      backgroundColor = QColor(247, 247, 247, 255)
      return {
        'backgroundBrush': parseBrush(backgroundColor, SolidFill, ),
        'textPen'        : parsePen(QColor(0, 0, 0, 191, ), 1, SolidLine, ),
        'borderBrush'    : parseBrush(QColor(0, 0, 0, 191, ), SolidFill, ),
      }
    backgroundColor = QColor(247, 247, 247, 255)
    return {
      'backgroundBrush': parseBrush(backgroundColor, SolidFill, ),
      'textPen'        : parsePen(QColor(0, 0, 0, 127, ), 1, SolidLine, ),
      'borderBrush'    : parseBrush(QColor(0, 0, 0, 127, ), SolidFill, ),
    }


class TopicSelection(Label):
  """TopicSelection provides topic selection functionality. A dropdown menu
  lists all available topics, a lineedit enables filtering, and a
  confirmation button locks the topic. Once locked, the topic can be changed
  with a clear button."""

  baseLayout: QGridLayout
  baseWidget: BaseWidget
  #  Widgets
  topicBanner: Label
  topicComboBox: TopicComboBox
  topicFilter: QLineEdit
  selectButton: _Button
  headerTopicName: Label
  selectedTopicName: Label
  headerTopicType: Label
  selectedTopicType: Label
  headerEcho: Label
  topicEcho: Label
  clearButton: _Button

  __current_topic__ = None

  rosTopic = EmptyField()

  def _getRosTopic(self) -> RosTopic:
    """Getter-function for the ROS topic"""

  def __init__(self, *args, **kwargs) -> None:
    data = kwargs | dict(id='topicSelectionBase')
    Label.__init__(self, *args, **data)

  def initUi(self) -> None:
    """Initialises the user interface"""
    #  Set the layout
    self.baseLayout = QGridLayout()
    self.baseLayout.setContentsMargins(1, 1, 1, 1, )
    self.baseLayout.setSpacing(0)
    #  Banner
    self.topicBanner = Label('Topic Selection Menu', id='title')
    self.topicBanner.initUi()
    self.baseLayout.addWidget(self.topicBanner, 0, 0, 1, 3)
    #  Clear button
    self.clearButton = _Button('Clear\nTopic', id='clearButton')
    self.clearButton.initUi()
    self.clearButton.setSizePolicy(Prefer, Prefer)
    self.clearButton.setMouseTracking(True)
    self.baseLayout.addWidget(self.clearButton, 1, 0, 1, 1)
    self.clearButton.setEnabled(False)
    #  Dropdown menu
    self.topicComboBox = TopicComboBox()
    self.topicComboBox.initUi()
    self.baseLayout.addWidget(self.topicComboBox, 1, 1, 1, 1)
    #  Select button
    self.selectButton = _Button('Select\nTopic', id='selectButton')
    self.selectButton.initUi()
    self.selectButton.setSizePolicy(Prefer, Prefer)
    self.selectButton.setMouseTracking(True)
    self.baseLayout.addWidget(self.selectButton, 1, 2, 1, 1)
    self.selectButton.setEnabled(True)
    #  Header topic name
    self.headerTopicName = Label('Topic Name:', id='topicHeader')
    self.headerTopicName.initUi()
    self.baseLayout.addWidget(self.headerTopicName, 2, 0, 1, 1)
    #  Selected topic name
    self.selectedTopicName = Label(' - ', id='selectedTopic')
    self.selectedTopicName.initUi()
    self.baseLayout.addWidget(self.selectedTopicName, 2, 1, 1, 2)
    #  Header topic type
    self.headerTopicType = Label('Topic Type:', id='topicHeader')
    self.headerTopicType.initUi()
    self.baseLayout.addWidget(self.headerTopicType, 3, 0, 1, 1)
    #  Selected topic type
    self.selectedTopicType = Label(' - ', id='selectedTopic')
    self.selectedTopicType.initUi()
    self.baseLayout.addWidget(self.selectedTopicType, 3, 1, 1, 2)
    #  Header echo
    self.headerEcho = Label('Topic Echo:', id='topicHeader')
    self.headerEcho.initUi()
    self.baseLayout.addWidget(self.headerEcho, 4, 0, 1, 1)
    #  Topic echo
    self.topicEcho = Label(' - ', id='selectedTopic')
    self.topicEcho.initUi()
    self.baseLayout.addWidget(self.topicEcho, 4, 1, 1, 2)
    #  Set the layout
    self.setLayout(self.baseLayout)

  def echoValue(self, data: Any) -> None:
    """Echoes the value of the topic"""
    msgType = self.topicComboBox.selectedType

  def initSignalSlot(self) -> None:
    """Initialises the signal-slot connections"""
    self.selectButton.singleClick.connect(self.topicComboBox.selectTopic)
    self.clearButton.singleClick.connect(self.topicComboBox.clearTopic)
    self.topicComboBox.initSignalSlot()
    self.topicComboBox.topicSelected.connect(self._setCurrentTopic)
    self.topicComboBox.topicCleared.connect(self._clearCurrentTopic)
    self.selectButton.singleClick.connect(lambda: ic('select'))
    self.clearButton.singleClick.connect(lambda: ic('clear'))

  @classmethod
  def staticStyles(cls) -> dict[str, Any]:
    """Returns the static styles"""
    labelStyles = Label.staticStyles()
    topicSelectionStyles = {
      'margins' : QMargins(2, 2, 2, 2, ),
      'borders' : QMargins(2, 2, 2, 2, ),
      'paddings': QMargins(2, 2, 2, 2, ),
    }
    return {**labelStyles, **topicSelectionStyles}

  def _setCurrentTopic(self, topicName: str) -> None:
    """Setter-function for the current topic"""
    self.__current_topic__ = topicName
    self.selectedTopicName.text = topicName
    topicType = self.topicComboBox.getTopicTypeDict()[topicName]['type_']
    if isinstance(topicType, type):
      self.selectedTopicType.text = topicType.__name__
    elif isinstance(topicType, str):
      self.selectedTopicType.text = topicType
    elif topicType is None:
      e = """Unable to determine the topic type for topic: '%s'!"""
      raise TypeError(monoSpace(e % topicName))
    else:
      e = typeMsg('topicType', topicType, type)
      raise TypeError(e)
    self.selectedTopicType.update()
    self.selectedTopicName.update()
    self.selectButton.setEnabled(False)
    self.clearButton.setEnabled(True)
    self.rosEcho = Subscriber(topicName, topicType, self._echoValue)

  def _echoValue(self, data: Any) -> None:
    """Echoes the value of the topic"""
    self.topicEcho.text = 'Echo: %.3f' % data.data
    self.topicEcho.update()

  def _getCurrentTopic(self) -> str:
    """Getter-function for the current topic"""
    return self.__current_topic__

  def _clearCurrentTopic(self) -> None:
    """Deleter-function for the current topic"""
    self.__current_topic__ = None
    self.selectedTopicName.text = 'No topic selected'
    self.selectedTopicType.text = 'No topic type'
    self.selectedTopicType.update()
    self.selectedTopicName.update()
    self.clearButton.setEnabled(False)
    self.selectButton.setEnabled(True)
