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
