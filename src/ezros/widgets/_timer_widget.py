"""TimerWidget wraps the QTimer and provides a widget interface allowing
users to stop and start the timer and even adjust the interval."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Signal
from attribox import AttriBox
from ezside.core import Precise
from ezside.widgets import BaseWidget

from ezros.widgets import SpinBoxFloat, PushButton, Horizontal, SpinBoxInt


class TimerWidget(BaseWidget):
  """TimerWidget wraps the QTimer and provides a widget interface allowing
  users to stop and start the timer and even adjust the interval."""

  now = None

  __timer_paused__ = False
  __paused_time__ = None

  # timer = AttriBox[QTimer]()
  baseLayout = AttriBox[Horizontal]()
  startButton = AttriBox[PushButton]('START')
  pauseButton = AttriBox[PushButton]('PAUSE')
  resumeButton = AttriBox[PushButton]('RESUME')
  stopButton = AttriBox[PushButton]('STOP')
  intervalInput = AttriBox[SpinBoxInt]('Interval', 1, 500, 1000)

  timeout = Signal()
  paused = Signal(float)
  started = Signal()
  stopped = Signal()

  def initUi(self, ) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.baseLayout.addWidget(self.startButton)
    self.baseLayout.addWidget(self.pauseButton)
    self.baseLayout.addWidget(self.resumeButton)
    self.baseLayout.addWidget(self.stopButton)
    self.baseLayout.addWidget(self.intervalInput)
    self.setLayout(self.baseLayout)
    self.connectActions()
    self.timer.setInterval(self.intervalInput.value)
    self.timer.setTimerType(Precise)
    self.timer.setSingleShot(False)

  def connectActions(self) -> None:
    """Initialize the actions."""
    self.startButton.clicked.connect(self.start)
    self.pauseButton.clicked.connect(self.pause)
    self.resumeButton.clicked.connect(self.resume)
    self.stopButton.clicked.connect(self.stop)
    self.intervalInput.valueChanged.connect(self.setInterval)
    self.timer.timeout.connect(self.timeout)

  def start(self) -> None:
    """Start the timer."""
    self.timer.start()
    self.started.emit()

  def pause(self) -> None:
    """Pause the timer."""
    self.__timer_paused__ = True
    self.__paused_time__ = self.timer.remainingTime()
    self.timer.stop()
    self.paused.emit(self.__paused_time__)

  def resume(self) -> None:
    """Resume the timer."""
    self.__timer_paused__ = False
    self.timer.start(self.__paused_time__)
    if self.__paused_time__ is None:
      raise RuntimeError('Timer is not paused.')
    resumeTimer = QTimer(self, self.__paused_time__, singleShot=True, )
    resumeTimer.timeout.connect(self.resume)
    resumeTimer.start()

  def stop(self) -> None:
    """Stop the timer."""
    self.timer.stop()
    self.__timer_paused__ = False
    self.stopped.emit()

  def setInterval(self, interval: int) -> None:
    """Set the interval."""
    self.__timer_paused__ = self.timer.remainingTime()
    self.timer.setInterval(interval)
    resumeTimer = QTimer()
    resumeTimer.setInterval(self.__timer_paused__)
    resumeTimer.timeout.connect(self.timer.start)
    resumeTimer.start()
