"""The MainWindow class organizes the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from ezros.app import (LayoutWindow)

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)

    self.setWindowTitle('EZROS')
    self.resize(800, 600)

  def debug1Func(self, ) -> None:
    LayoutWindow.debug1Func(self, )
    x = self.dynChart.dataView.innerChart.axes()[0].range()
    y = self.dynChart.dataView.innerChart.axes()[1].range()
    ic(x, y)
