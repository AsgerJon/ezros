"""App subclasses the QApplication class."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezside.app import App


class EZRos(App):
  """App is a subclass of QApplication."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the App instance."""
    App.__init__(self, *args, **kwargs)
    self.setApplicationName('EZROS')
    self.setApplicationDisplayName('EZROS')
