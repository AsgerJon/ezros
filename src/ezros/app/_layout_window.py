"""The LayoutWindow extends the BaseWindow from vistside."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.core import parseParent
from vistside.windows import BaseWindow

from ezros.gui import ClientInfoField


class LayoutWindow(BaseWindow):
  """The LayoutWindow class is a window that contains a layout of widgets."""

  clientInfo = ClientInfoField()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new LayoutWindow."""
    BaseWindow.__init__(self, )

  def show(self) -> None:
    """Show the window."""
    self.baseLayout.addWidget(self.clientInfo)
    BaseWindow.show(self)