"""ClientInfo provides static information about the client."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any, Never

from PySide6.QtGui import QColor, QPaintEvent, QPainter
from vistside.core import BrushField, parsePen, Black, Tight
from vistside.widgets import BaseWidget, BaseLayoutField, LabelField, \
  LabelWidget
from vistutils.fields import unParseArgs, Wait, FieldBox


class ClientInfo(BaseWidget):
  """The ClientInfo class provides static information about the client."""

  __fallback_uri__ = 'http://localhost:11311'
  __ros_master_uri__ = os.environ.get('ROS_MASTER_URI', __fallback_uri__)

  fillBrush = BrushField(QColor(255, 255, 255, 63))

  baseLayout = BaseLayoutField(layout='vertical')

  clientTitle = FieldBox[LabelWidget]('Client Info: ')
  clientURI = FieldBox[LabelWidget]('URI: ')
  clientLocation = FieldBox[LabelWidget]('Location: ')

  def __init__(self, *args, **kwargs) -> None:
    """Create a new ClientInfo."""
    BaseWidget.__init__(self, *args, **kwargs)
    self.clientTitle.innerText += 'Sports Complex'
    uri = self.__ros_master_uri__.replace('http://', '')
    uri = uri.replace(':11311', '')
    self.clientURI.innerText += uri
    self.clientLocation.innerText += 'Somewhere'
    self.initUi()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.clientTitle)
    self.baseLayout.addWidget(self.clientURI)
    self.baseLayout.addWidget(self.clientLocation)
    self.setLayout(self.baseLayout)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    viewRect = painter.viewport()
    # # # # # # # # # # # # # # # # #
    #  fill background
    painter.setPen(self.emptyPen)
    painter.setBrush(self.fillBrush)
    painter.drawRect(viewRect)
    # # # # # # # # # # # # # # # # #
    #  draw border
    painter.setPen(parsePen(Black, 1))
    painter.setBrush(self.emptyBrush)
    painter.drawRect(viewRect)
    painter.end()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> ClientInfo:
    """Returns the default value for the field."""
    return cls(*args, **kwargs)

  def apply(self, value: Any) -> ClientInfo:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ClientInfoField(Wait):
  """ClassInfoField provides static information about the class."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the descriptor"""
    Wait.__init__(self, ClientInfo, *args, **kwargs)
