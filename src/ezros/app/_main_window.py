"""The MainWindow class organizes the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.app import LayoutWindow


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)

    self.setWindowTitle('EZROS')
    self.resize(800, 600)

  def connectActions(self) -> None:
    """Connect the actions."""
    self.testButton.singleClick.connect(self.onTestButtonClicked)

  def onTestButtonClicked(self, ) -> None:
    """Handle the test button clicked event."""
    text = self.welcomeBanner.getText()
    self.welcomeBanner.setText('[> %s <]' % text)
    self.welcomeBanner.update()
