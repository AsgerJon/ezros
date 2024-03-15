"""ConnectionStatus shows the present connection status of the robot. In
particular the ping time. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.widgets import LabelWidget
from PySide6.QtGui import QColor, QPainter, QPaintEvent
from vistside.core import solidBrush
from vistside.widgets import BaseWidget, BaseLayoutField
from vistutils.fields import FieldBox

from ezros.gui import OpState, PingIndicator
from ezros.rosutils._wait import WaitForIt


class ConnectionStatus(BaseWidget):
  """ConnectionStatus shows the present connection status of the robot. In
  particular the ping time. """

  fillColor = FieldBox[QColor](0, 0, 255, 63)
  baseLayout = BaseLayoutField(layout='vertical')

  headerLabel = FieldBox[LabelWidget]()
  pingIndicator = FieldBox[PingIndicator]()
  operationalState = FieldBox[OpState]

  def __init__(self, *args, **kwargs) -> None:
    """Create a new OpState."""
    BaseWidget.__init__(self, *args, **kwargs)
    # self.initUI()

  def initUI(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.headerLabel)
    self.baseLayout.addWidget(self.pingIndicator)
    self.baseLayout.addWidget(self.operationalState)
    self.setLayout(self.baseLayout)
    with WaitForIt() as yolo:
      print(__file__)
      yolo.run_code(self)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    viewRect = painter.viewport()
    painter.setPen(self.emptyPen)
    painter.setBrush(solidBrush(self.fillColor))
    painter.drawRoundedRect(viewRect, 10, 10)
    painter.end()
