"""MainWindow subclasses the LayoutWindow and provides the main
application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.widgets import BaseWidget
from vistutils.fields import Wait

from ezros.app import LayoutWindow


class MainWindow(LayoutWindow):
  """The MainWindow class is the main application window."""

  baseWidget = Wait(BaseWidget)
  baseLayout = Wait(BaseLayout)

  def __init__(self, *args, **kwargs) -> None:
    """Create a new MainWindow."""
    LayoutWindow.__init__(self, *args, **kwargs)
