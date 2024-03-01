"""SubMain is the main application window for the subscriber class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.sub import SubLayout


class SubMain(SubLayout):
  """SubMain is the main application window for the subscriber class"""

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.setWindowTitle("Subscriber")
