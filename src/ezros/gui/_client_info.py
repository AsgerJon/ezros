"""ClientInfo provides static information about the client."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any

from PySide6.QtGui import QColor, QPaintEvent, QPainter
from vistside.core import BrushField, parsePen, Black
from vistside.widgets import BaseWidget, BaseLayoutField, LabelField
from vistutils.fields import unParseArgs, Wait


class ClientInfo(BaseWidget):
  """The ClientInfo class provides static information about the client."""

  __fallback_uri__ = 'http://localhost:11311'
  __ros_master_uri__ = os.environ.get('ROS_MASTER_URI', __fallback_uri__)
  fillBrush = BrushField(QColor(255, 255, 255, 63))

  baseLayout = BaseLayoutField(layout='vertical')
  rosUriField = LabelField('ROS Master URI: ')
  clientInfo = LabelField('Client Info:')
  clientLocation = LabelField('Location: ')

  def __init__(self, *args, **kwargs) -> None:
    """Create a new ClientInfo."""
    BaseWidget.__init__(self, *args, **kwargs)
    self.rosUriField = self.__ros_master_uri__
    clientName = kwargs.get('client', None)
    self.clientInfo = clientName
    location = kwargs.get('location', None)
    self.clientLocation = location
    self.initUi()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.rosUriField)
    self.baseLayout.addWidget(self.clientInfo)
    self.baseLayout.addWidget(self.clientLocation)
    self.setLayout(self.baseLayout)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(painter.Antialiasing)
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
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> ClientInfo:
    """Applies the arguments contained in value to the widget."""
    args, kwargs = unParseArgs(value)
    strArgs = [*[arg for arg in args if isinstance(arg, str)], None, None]
    name, location = strArgs[:2]
    if name is not None:
      self.clientInfo = name
    if location is not None:
      self.clientLocation = location
    return self


class ClientInfoField(Wait):
  """ClassInfoField provides static information about the class."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the descriptor"""
    Wait.__init__(self, ClientInfo, *args, **kwargs)
