"""The MainWindow class organizes the main application window."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from rospy import init_node
from rospy.core import is_initialized

from ezros.app import LayoutWindow

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def initSignalSlot(self) -> None:
    """Initialize the signal-slot connections."""
    self.debug1.triggered.connect(self.debug1Func)
    self.debug2.triggered.connect(self.debug2Func)
    self.debug3.triggered.connect(self.debug3Func)
    self.debug4.triggered.connect(self.debug4Func)
    self.debug5.triggered.connect(self.debug5Func)
    self.debug6.triggered.connect(self.debug6Func)
    self.debug7.triggered.connect(self.debug7Func)
    self.debug8.triggered.connect(self.debug8Func)
    self.debug9.triggered.connect(self.debug9Func)

  def debug1Func(self, ) -> None:
    """Debug1 function."""
    note = 'Debug1 function called, add data to chart view'
    self.statusBar().showMessage(note)

  def debug2Func(self, ) -> None:
    """Debug2 function."""
    note = 'Debug2 function called'
    self.statusBar().showMessage(note)
    ic(is_initialized())

  def debug3Func(self, ) -> None:
    """Debug3 function."""
    note = 'Debug3 function called'
    print(note)
    self.statusBar().showMessage(note)
    init_node('fuck you!', anonymous=True)

  def debug4Func(self, ) -> None:
    """Debug4 function."""
    note = 'Debug4 function called'
    print(note)
    self.statusBar().showMessage(note)

  def debug5Func(self, ) -> None:
    """Debug5 function."""
    note = 'Debug5 function called'
    print(note)
    self.statusBar().showMessage(note)

  def debug6Func(self, ) -> None:
    """Debug6 function."""
    note = 'Debug6 function called'
    print(note)
    self.statusBar().showMessage(note)

  def debug7Func(self, ) -> None:
    """Debug7 function."""
    note = 'Debug7 function called'
    print(note)
    self.statusBar().showMessage(note)

  def debug8Func(self, ) -> None:
    """Debug8 function."""
    note = 'Debug8 function called'
    print(note)
    self.statusBar().showMessage(note)

  def debug9Func(self, ) -> None:
    """Debug9 function."""
    note = 'Debug9 function called'
    print(note)
    self.statusBar().showMessage(note)
