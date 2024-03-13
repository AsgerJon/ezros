"""TestPlot simplifies the QCharts for debugging"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget


class TestPlot(QWidget):
  """The TestPlot class provides a simple plot for debugging."""

  def __init__(self, *args, **kwargs) -> None:
    self.series = QLineSeries
