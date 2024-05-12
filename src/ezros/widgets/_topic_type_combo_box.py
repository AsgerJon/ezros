"""TopicTypeComboBox provides a combo box listing the unique types of
topics currently published on the ROS master. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Self

from PySide6.QtWidgets import QComboBox
from rospy import get_published_topics
from rostopic import get_topic_class

from ezros.widgets import TopicComboBox


class TopicTypeComboBox(TopicComboBox):
  """TopicTypeComboBox provides a combo box listing the unique types of
  topics currently published on the ROS master. """

  def __init__(self, *args, ) -> None:
    TopicComboBox.__init__(self, *args)
    self.lineEdit().setPlaceholderText('Select a topic type...')

  def resetTopics(self) -> None:
    """Resets the topics listed. """
    topicList = []
    for topicName, topicType in get_published_topics():
      topicType = get_topic_class(topicName)
      for item in topicType:
        if isinstance(item, type):
          break
      else:
        continue
      if item not in topicList:
        topicList.append(item)
    topicList.sort()
    self.setInsertPolicy(QComboBox.InsertPolicy.InsertAtBottom)
    for topicType in topicList:
      typeName = topicType.__name__
      self.addItem(typeName, topicType)
    self.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)

  def item(self, index: int) -> type:
    """Returns the current item. """
    return QComboBox.itemData(self, index)

  def currentItem(self) -> type:
    """Returns the current item. """
    return self.itemData(self.currentIndex())

  def __iter__(self, ) -> Self:
    """Implementation of the iterator protocol. """
    self.__iter_contents__ = [self.item(i) for i in range(self.count())]
    return self

  def __next__(self, ) -> type:
    """Implementation of the next method. """
    if self.__iter_contents__:
      return self.__iter_contents__.pop(0)
    else:
      raise StopIteration
