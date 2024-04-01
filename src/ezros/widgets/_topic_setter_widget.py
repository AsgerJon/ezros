"""TopicSetterWidget provides an input widget for topic name."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class TopicSetterWidget(BaseWidget):
  """TopicSetterWidget provides an input widget for topic name."""

  baseLayout = AttriBox[Grid]()
  topicLabel = AttriBox[TightLabel]()
