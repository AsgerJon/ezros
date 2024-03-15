"""The LayoutWindow extends the BaseWindow from vistside."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QHBoxLayout
from vistside.widgets import BaseLayoutField, BaseWidget
from vistside.windows import BaseWindow
from vistutils.fields import FieldBox

from ezros.gui import ClientInfo, TabWidget, ConnectionStatus
from ezros.rosutils import WaitForIt

tabWidget = FieldBox[TabWidget]()


class LayoutWindow(BaseWindow):
  """The LayoutWindow class is a window that contains a layout of widgets."""

  topWidget = FieldBox[BaseWidget]()
  tabWidget = FieldBox[TabWidget]()
  topLayout = FieldBox[QHBoxLayout]()
  clientInfo = FieldBox[ClientInfo]()
  connectionStatus = FieldBox[ConnectionStatus]()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new LayoutWindow."""
    BaseWindow.__init__(self, *args, **kwargs, )

  def show(self) -> None:
    """Show the window."""
    with WaitForIt() as yolo:
      yolo.run_code(self)
    self.topLayout.addWidget(self.clientInfo)
    self.topLayout.addWidget(self.connectionStatus)
    self.topWidget.setLayout(self.topLayout)
    self.baseLayout.addWidget(self.topWidget)
    self.baseLayout.addWidget(self.tabWidget)
    BaseWindow.show(self)
