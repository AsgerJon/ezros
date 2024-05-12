"""TopicComboBox provides a QComboBox subclass automatically populated
with the available topics on the ROS master, subject to some filtering. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Self

from PySide6.QtWidgets import QComboBox, QWidget
from icecream import ic
from rospy import get_published_topics

from ezros.rosutils import RosTopic

ic.configureOutput(includeContext=True)


class TopicComboBox(QComboBox):
  """TopicComboBox provides a QComboBox subclass automatically populated
  with the available topics on the ROS master, subject to some filtering. """

  __iter_contents__ = None

  def __init__(self, *args) -> None:
    """Initializes the TopicComboBox instance. """
    for arg in args:
      if isinstance(arg, QWidget):
        QComboBox.__init__(self, arg)
        break
    else:
      QComboBox.__init__(self, )
    self.resetTopics()
    self.setEditable(True)
    self.lineEdit().setPlaceholderText('Select a topic...')

  def resetTopics(self) -> None:
    """Resets the topics listed. """
    topicList = []
    for topicName, topicType in get_published_topics():
      topicList.append(topicName)
    topicList.sort()
    self.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    self.addItems(topicList)
    self.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

  def addItem(self, topicName: str, *args) -> None:
    """Adds an item to the combobox. """
    rosTopic = RosTopic(topicName)
    QComboBox.addItem(self, topicName, rosTopic)

  def addItems(self, topicNames: list[str]) -> None:
    """Adds items to the combobox. """
    for topicName in topicNames:
      self.addItem(topicName)

  def item(self, index: int) -> RosTopic:
    """Returns the current item. """
    return QComboBox.itemData(self, index)

  def currentItem(self) -> RosTopic:
    """Returns the current item. """
    return self.itemData(self.currentIndex())

  def __iter__(self, ) -> Self:
    """Implementation of the iterator protocol. """
    self.__iter_contents__ = [self.item(i) for i in range(self.count())]
    return self

  def __next__(self, ) -> RosTopic:
    """Implementation of the next method. """
    if self.__iter_contents__:
      return self.__iter_contents__.pop(0)
    else:
      raise StopIteration
