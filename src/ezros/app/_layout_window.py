"""The LayoutWindow extends the BaseWindow from vistside."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.widgets import BaseLayoutField, BaseWidget
from vistside.windows import BaseWindow
from vistutils.fields import Wait, FieldBox
from vistutils.fields._break_point import BreakPoint

from ezros.gui import ClientInfoField, ConnectionStatusField, TabWidget

tabWidget = FieldBox[TabWidget]()


class LayoutWindow(BaseWindow):
  """The LayoutWindow class is a window that contains a layout of widgets."""

  topWidget = FieldBox[BaseWidget]()
  tabWidget = FieldBox[TabWidget]()
  topLayout = BaseLayoutField(layout='horizontal')
  clientInfo = ClientInfoField()
  connectionStatus = ConnectionStatusField()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new LayoutWindow."""
    BaseWindow.__init__(self, *args, **kwargs, )

  def show(self) -> None:
    """Show the window."""
    self.topLayout.addWidget(self.clientInfo)
    self.topLayout.addWidget(self.connectionStatus)
    self.topWidget.setLayout(self.topLayout)
    self.baseLayout.addWidget(self.topWidget)
    self.baseLayout.addWidget(self.tabWidget)
    BaseWindow.show(self)
