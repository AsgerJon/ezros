"""PubMain provides the business logic of the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
from rospy import get_time, Publisher
from std_msgs.msg import Float64

from ezros.gui.factories import timerFactory
from ezros.pub import PubLayoutWindow
from morevistutils import Wait

from math import sin
from random import gauss

ic.configureOutput(includeContext=True)


class PubMain(PubLayoutWindow):
  """The main application window"""

  sineTest = None
  sineTimer = Wait(timerFactory(), 33, singleShot=False)

  @staticmethod
  def sineSample(t: float = None) -> Float64:
    """Generates a sine wave with added Gaussian noise"""
    if t is None:
      t = get_time()
    F = [1, 4, 9, 16, 25, ]
    R = [1, 1 / 2, 1 / 6, 1 / 24, 1 / 120]
    val = sum([r * sin(f * t) for (f, r) in zip(F, R)])
    val += gauss(0, 0.1)
    out = Float64()
    out.data = val
    return out

  def createActionStub(self) -> None:
    """Omitting..."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the main application window"""
    PubLayoutWindow.__init__(self, *args, **kwargs)
    self.sineTest = Publisher('topic', Float64, queue_size=10)

  def connectActions(self) -> None:
    """Initializes the user interface"""
    self.sineTimer.timeout.connect(self.publishSample)
    self.sineTimer.timeout.connect(self.updateLabel)
    self.sineTimer.start()

  def publishSample(self, ) -> None:
    """Publishes a sample sine wave"""
    sample = self.sineSample()
    self.sineTest.publish(sample)

  def updateLabel(self, msg: Float64 = None) -> None:
    """Updates the label with the latest sample"""
    self.label.innerText = f'{msg.data:.3f}'
    self.label.update()
