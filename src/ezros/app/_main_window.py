"""MainWindow subclasses the LayoutWindow and provides the main
application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import random
import time
from math import sin

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QInputDialog
from vistside.core import TimerField, Precise, generateWhiteNoise
from vistside.windows import BaseWindow

from ezros.app import LayoutWindow


class MainWindow(LayoutWindow, ):
  """The MainWindow class is the main application window."""

  dataTimer = TimerField(50, Precise, singleShot=False)
  paintTimer = TimerField(50, Precise, singleShot=False)

  noise = Signal(float)

  def __init__(self, *args, **kwargs) -> None:
    """Create a new MainWindow."""
    LayoutWindow.__init__(self, *args, **kwargs)
    self._noiseBank = generateWhiteNoise(1000, 100)
    self.__current_index__ = 0
    self.__debug_command__ = None

  def connectActions(self, ) -> None:
    """Connect the actions to the slots."""
    LayoutWindow.connectActions(self)
    self.dataTimer.timeout.connect(self.makeNoise)
    self.noise.connect(self.callback)
    self.paintTimer.timeout.connect(self.paintBack)

  def makeNoise(self, ) -> float:
    """Make some noise."""
    t = time.time()
    out = (sin(t) * 4 + sin(8 * t) * 2 + sin(64 * t) * 1) / 7
    out += (random.random() - 0.5) * 0.1
    self.noise.emit(out)
    return out

  @Slot(float)
  def callback(self, value) -> None:
    """Appends new data (value, timestamp)."""
    self.tabWidget.callback(value)

  @Slot()
  def paintBack(self, ) -> None:
    """Updates the plot visualization."""
    self.tabWidget.plotWidget.updateChart()

  def debug01Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug01Func(self)
    print('start of data timer')
    self.dataTimer.start()
    self.mainStatusBar.showMessage('Debug01 action triggered!')

  def debug02Func(self) -> None:
    """Debug function."""
    BaseWindow.debug02Func(self)
    print('Debug02 | start paint timer')
    self.paintTimer.start()
    self.mainStatusBar.showMessage('Debug02 action triggered!')

  def debug03Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug03Func(self)
    print('force repaint of tab widget')
    self.tabWidget.repaint()
    self.mainStatusBar.showMessage('Debug03 action triggered!')

  def debug04Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug04Func(self)
    print('Debug04 | force repaint of plot widget')
    self.tabWidget.plotWidget.update()
    self.mainStatusBar.showMessage('Debug04 action triggered!')

  def debug05Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug05Func(self)
    print('Debug05 | show current data content')
    print(self.tabWidget.plotWidget.data)
    self.mainStatusBar.showMessage('Debug05 action triggered!')

  def debug06Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug06Func(self)
    print('Debug06 | show current data content')
    cmd, lmao = QInputDialog.getText(self, 'input command', 'LOL')
    self.mainStatusBar.showMessage('Debug06 action triggered!')
    self.__debug_command__ = cmd
    print(self.__debug_command__)

  def debug09Func(self, ) -> None:
    """Debug function."""
    BaseWindow.debug09Func(self)
    print('running %s!' % self.__debug_command__)
    exec(self.__debug_command__)
    self.mainStatusBar.showMessage('Debug09 action triggered!')
