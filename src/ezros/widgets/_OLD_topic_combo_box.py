"""TopicComboBox provides a QComboBox subclass automatically populated
with the available topics on the ROS master, subject to some filtering. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Slot, Signal
from PySide6.QtWidgets import QComboBox, QVBoxLayout
from ezside.widgets import BaseWidget
from icecream import ic
from rospy import get_published_topics, Subscriber
from rostopic import get_topic_type, get_topic_class
from vistutils.fields import EmptyField
from vistutils.waitaminute import typeMsg

from ezros.rosutils import RosTopic, RosObject

ic.configureOutput(includeContext=True)


class TopicComboBox(BaseWidget):
  """TopicComboBox provides a QComboBox subclass automatically populated
  with the available topics on the ROS master, subject to some filtering. """

  baseLayout: QVBoxLayout
  nameComboBox: QComboBox
  typeComboBox: QComboBox
  rosTopic: RosTopic

  __topic_list__ = None
  __topic_dict__ = None
  __iter_contents__ = None

  __selected_topic__ = None

  topicSelected = Signal(str)  # Emitted by the select button - hard
  currentTopicChanged = Signal(str)  # Emitted by the combobox - soft
  topicCleared = Signal()
  echo = Signal(str)
  info = Signal(str)

  selectedTopic = EmptyField()
  selectedType = EmptyField()

  @selectedTopic.GET
  def getSelectedTopic(self) -> str:
    """Getter-function for the selected topic. """
    return self.__selected_topic__

  @selectedType.GET
  def getSelectedType(self) -> type:
    """Getter-function for the selected topic type. """
    if self.selectedTopic is not None:
      return get_topic_type(self.selectedTopic)[0]

  @classmethod
  def styleTypes(cls) -> dict[str, type]:
    """Returns the style types. """
    return {}

  @classmethod
  def staticStyles(cls, ) -> dict[str, Any]:
    """Returns the static styles. """
    return {}

  def dynStyles(self) -> dict[str, Any]:
    """Returns the dynamic styles. """
    return {}

  def initUi(self) -> None:
    """Initializes the user interface. """

    #  Set the layout
    self.baseLayout = QVBoxLayout()
    self.baseLayout.setContentsMargins(0, 0, 0, 0)
    self.baseLayout.setSpacing(0)
    #  Append the combo box
    self.nameComboBox = QComboBox()
    self.nameComboBox.setEditable(True)
    self.nameComboBox.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
    self.nameComboBox.lineEdit().setPlaceholderText('Select Topic')
    self.baseLayout.addWidget(self.nameComboBox)
    #  Append type combo box
    self.typeComboBox = QComboBox()
    self.typeComboBox.setEditable(True)
    self.typeComboBox.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
    self.typeComboBox.lineEdit().setPlaceholderText('Select Type')
    self.baseLayout.addWidget(self.typeComboBox)
    self.refreshTopicNameComboBox()
    self.refreshTopicTypeComboBox()
    #  Set the layout
    self.setLayout(self.baseLayout)

  def initSignalSlot(self) -> None:
    """Initializes the signal-slot connections. """
    self.nameComboBox.currentTextChanged.connect(self.currentTopicChanged)
    self.nameComboBox.currentTextChanged.connect(self._matchTopicType)

  def refreshTopics(self) -> None:
    """Updates the topic dictionary"""
    self.__topic_dict__ = {}
    for (name, type_) in get_published_topics():
      rosType, rosName = [*get_topic_class(name), None, None][:2]
      self.__topic_dict__[name] = dict(name=rosName, type_=rosType)

  def refreshTopicNameComboBox(self) -> None:
    """Updates the topic dictionary. This also removes filters."""
    self.nameComboBox.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    self.nameComboBox.clear()
    self.refreshTopics()
    for (key, val) in self.__topic_dict__.items():
      self.nameComboBox.addItem(val['name'])
    self.nameComboBox.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

  def refreshTopicTypeComboBox(self) -> None:
    """Updates the topic type combo box. """
    self.typeComboBox.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    self.typeComboBox.clear()
    self.refreshTopics()
    for (key, val) in self.__topic_dict__.items():
      self.typeComboBox.addItem(val['type_'].__name__)
    self.typeComboBox.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

  def applyNameFilter(self, filter_: str) -> None:
    """Filters the items in the combobox to topics with names that include
    the given filter string. """
    self.nameComboBox.clear()
    for (key, val) in self.__topic_dict__.items():
      if filter_ in key:
        self.nameComboBox.addItem(key)

  def applyTypeFilter(self, filter_: type):
    """Filters the items in the combobox to topics of the given type."""
    self.nameComboBox.clear()
    for (key, val) in self.__topic_dict__.items():
      if filter_ == val:
        self.nameComboBox.addItem(key)

  def getTopicTypes(self) -> list[type]:
    """Returns a list of the topic types in the combobox."""
    topicTypes = []
    for (key, val) in self.__topic_dict__.items():
      if val not in topicTypes:
        topicTypes.append(val)
    return topicTypes

  def getTopicTypeDict(self) -> dict[str, dict[str, type]]:
    """Returns a dictionary of the topic types in the combobox."""
    self.refreshTopics()
    return self.__topic_dict__

  def _setSelectedTopic(self, topic: str) -> None:
    """Setter-function for the selected topic. """
    self.__selected_topic__ = topic

  def _clearSelectedTopic(self) -> None:
    """Deleter-function for the selected topic. """
    self.__selected_topic__ = None

  def _lockTopic(self) -> None:
    """Locks the topic combo box. """
    topicName = self.nameComboBox.currentText()
    self._setSelectedTopic(topicName)
    self._matchTopicType()
    self.nameComboBox.setEnabled(False)
    self.typeComboBox.setEnabled(False)
    self.rosTopic = RosTopic(topicName)
    self.rosTopic.qRos.echo.connect(self.echo.emit)

  def _unlockTopic(self) -> None:
    """Unlocks the topic combo box. """
    self.info.emit('_unlockTopic')
    self._clearSelectedTopic()
    self.nameComboBox.setEnabled(True)
    self.typeComboBox.setEnabled(True)
    del self.rosTopic

  def _matchTopicType(self) -> None:
    """Matches the topic type. """
    self.refreshTopics()
    topicName = self.nameComboBox.currentText()
    ic(topicName)
    topicType = self.__topic_dict__.get(topicName, None)
    self.typeComboBox.setCurrentText(topicType['type_'].__name__)

  @Slot()
  def selectTopic(self) -> None:
    """Selects the topic. """
    self._lockTopic()
    self.topicSelected.emit(self.getSelectedTopic())

  @Slot()
  def clearTopic(self) -> None:
    """Clears the selected topic. """
    self._unlockTopic()
    self.topicCleared.emit()
