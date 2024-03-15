"""ConnectionStatus shows the present connection status of the robot. In
particular the ping time. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Self, Any

from vistside.widgets import LabelField, LabelWidget
from PySide6.QtGui import QColor, QPainter, QPaintEvent
from vistside.core import BrushField
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import Wait, FieldBox

from ezros.gui import PingIndicatorField, OpStateField, PingIndicator


class ConnectionStatus(BaseWidget):
  """ConnectionStatus shows the present connection status of the robot. In
  particular the ping time. """

  fillBrush = BrushField(QColor(0, 0, 255, 63))

  baseLayout = BaseLayoutField(layout='vertical')

  headerLabel = FieldBox[LabelWidget]('Connection Status')
  pingIndicator = FieldBox[PingIndicator]()
  operationalState = OpStateField()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new OpState."""
    BaseWidget.__init__(self, *args, **kwargs)
    self.headerLabel.innerText = 'Connection Status'
    self.initUI()

  def initUI(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.headerLabel)
    self.baseLayout.addWidget(self.pingIndicator)
    self.baseLayout.addWidget(self.operationalState)
    self.setLayout(self.baseLayout)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    viewRect = painter.viewport()
    painter.setPen(self.emptyPen)
    painter.setBrush(self.fillBrush)
    painter.drawRoundedRect(viewRect, 10, 10)
    painter.end()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> Self:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> Self:
    """Applies the arguments contained in value to the widget."""
    return self


class ConnectionStatusField(Wait):
  """The ConnectionStatusField class provides a descriptor for instances of
  ConnectionStatus."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the ConnectionStatusField."""
    Wait.__init__(self, ConnectionStatus, *args, **kwargs)
