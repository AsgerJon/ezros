"""TopicComboBox provides a QComboBox subclass automatically populated
with the available topics on the ROS master, subject to some filtering. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Slot, Signal
from PySide6.QtWidgets import QComboBox, QVBoxLayout
from ezside.widgets import BaseWidget
from rospy import get_published_topics
from rostopic import get_topic_type
from vistutils.fields import EmptyField
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg


class TopicComboBox(BaseWidget):
  """TopicComboBox provides a QComboBox subclass automatically populated
  with the available topics on the ROS master, subject to some filtering. """

  baseLayout: QVBoxLayout
  nameComboBox: QComboBox
  typeComboBox: QComboBox

  __topic_list__ = None
  __topic_dict__ = None
  __iter_contents__ = None

  __selected_topic__ = None

  topicSelected = Signal(str)  # Emitted by the select button - hard
  currentTopicChanged = Signal(str)  # Emitted by the combobox - soft
  topicCleared = Signal()

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
      topicType = self.__topic_dict__.get(self.selectedTopic, None)
      if isinstance(topicType, type):
        return topicType
      if topicType is not None:
        e = typeMsg('topicType', topicType, type)
        raise TypeError(e)

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

  def refreshTopicNameComboBox(self) -> None:
    """Updates the topic dictionary. This also removes filters."""
    self.nameComboBox.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    self.__topic_dict__ = {}
    self.nameComboBox.clear()
    for (name, type_) in get_published_topics():
      topicType = get_topic_type(name)[0]
      if topicType is None:
        continue
      self.__topic_dict__[name] = topicType
      self.nameComboBox.addItem(name)
    self.nameComboBox.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

  def refreshTopicTypeComboBox(self) -> None:
    """Updates the topic type combo box. """
    self.typeComboBox.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    self.typeComboBox.clear()
    topicTypes = self.getTopicTypes()
    for topicType in topicTypes:
      if isinstance(topicType, type):
        self.typeComboBox.addItem(topicType.__name__)
      elif isinstance(topicType, str):
        self.typeComboBox.addItem(topicType)
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

  def getTopicTypeDict(self) -> dict[str, type]:
    """Returns a dictionary of the topic types in the combobox."""
    return self.__topic_dict__

  def getSelectedTopic(self) -> str | None:
    """Getter-function for the selected topic. """
    return self.__selected_topic__

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

  def _unlockTopic(self) -> None:
    """Unlocks the topic combo box. """
    self._clearSelectedTopic()
    self.nameComboBox.setEnabled(True)
    self.typeComboBox.setEnabled(True)

  def _matchTopicType(self) -> None:
    """Matches the topic type. """
    topicName = self.nameComboBox.currentText()
    topicType = self.__topic_dict__.get(topicName, None)
    if topicType is None:
      e = """Unable to obtain the topic type for topic named: '%s'!"""
      raise KeyError(monoSpace(e % topicName))
    if isinstance(topicType, type):
      topicType = topicType.__name__
    for i in range(self.typeComboBox.count()):
      if self.typeComboBox.itemText(i) == topicType:
        self.typeComboBox.setCurrentIndex(i)
        return
    e = """Unable to find the topic type: '%s' of topic named: '%s' in the 
      type combo box!"""
    name, type_ = topicName, topicType.__name__
    raise KeyError(monoSpace(e % (name, type_)))

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
