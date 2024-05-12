"""The MainWindow class organizes the main application window."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic
import rospy

from ezros.app import LayoutWindow

# from msgs.msg import Float32Stamped

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  def initSignalSlot(self) -> None:
    """Initialize the signal-slot connections."""
    self.debug1.triggered.connect(self.onDebug1)
    self.debug2.triggered.connect(self.onDebug2)
    self.debug3.triggered.connect(self.onDebug3)
    self.topicSelection.topicComboBox.info.connect(self.onDebug1)

  def onDebug1(self, *args) -> None:
    """Debug1 action."""
    text = [*args, ''][0]
    print('info text: %s' % text)
    self.mainStatusBar.showMessage('Debug1 action triggered.', 5000)
    self.topicSelection.echoValue(7777)

  def onDebug2(self) -> None:
    """Debug2 action."""
    self.mainStatusBar.showMessage('Debug2 action triggered.', 5000)
    for item in dir(rospy.topics):
      ic(item)

  def onDebug3(self) -> None:
    """Debug3 action."""
    self.mainStatusBar.showMessage('Debug3 action triggered.', 5000)
    for (key, val) in rospy.__dict__.items():
      ic(key, val)
