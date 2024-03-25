"""TabWidget organizes the functionalities into tabs"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QTabWidget
from icecream import ic

ic.configureOutput(includeContext=True)


class TabWidget(QTabWidget):
  """TabWidget organizes the functionalities into tabs."""
