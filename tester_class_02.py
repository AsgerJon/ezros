"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.windows import BaseWindow

from tester_class_01 import TestField


class TestWindow(BaseWindow):
  """The TestWindow class is a window that contains a layout of widgets."""

  testWidget = TestField()

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestWindow."""
    BaseWindow.__init__(self, *args, **kwargs)
    self.setMinimumSize(800, 600)

  def show(self) -> None:
    """Show the window."""
    self.baseLayout.addWidget(self.testWidget)
    BaseWindow.show(self)
