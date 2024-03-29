"""The LayoutWindow extends the BaseWindow from vistside."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.widgets import BaseLayoutField, BaseWidget
from vistside.windows import BaseWindow
from vistutils.fields import Wait, FieldBox

from ezros.gui import ClientInfoField, ConnectionStatusField, TabWidget


class LayoutWindow(BaseWindow):
  """The LayoutWindow class is a window that contains a layout of widgets."""

  topWidget = Wait(BaseWidget, )
  topLayout = BaseLayoutField(layout='horizontal')
  clientInfo = ClientInfoField()
  connectionStatus = ConnectionStatusField()

  # tabWidget = FieldBox[TabWidget]()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new LayoutWindow."""
    BaseWindow.__init__(self, *args, **kwargs, )
    self.tabWidget = TabWidget(self)

  def show(self) -> None:
    """Show the window."""
    self.topLayout.addWidget(self.clientInfo)
    self.topLayout.addWidget(self.connectionStatus)
    self.topWidget.setLayout(self.topLayout)
    self.baseLayout.addWidget(self.topWidget)
    self.tabWidget.initUI()
    self.baseLayout.addWidget(self.tabWidget)
    BaseWindow.show(self)
