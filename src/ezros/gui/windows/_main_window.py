"""MainWindow provides the main application window. The BaseWindow class
provides menus and actions. The Layout Window class provides the layout of
widget that appear on the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

import rospy
from PySide6.QtCore import QTimer
from rospy import Subscriber, Publisher
from icecream import ic
from std_msgs.msg import Float64
from vistutils.text import monoSpace

from ezros.gui.factories import header
from ezros.gui.shortnames import Precise
from ezros.gui.windows import LayoutWindow, BaseWindow

from msgs.msg import AuxCommand, Float32Stamped

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """MainWindow provides the main application window. The BaseWindow class
  provides menus and actions. The Layout Window class provides the layout of
  widget that appear on the main application window"""

  __pump_control_flag__ = None
  __pump_control_timer__ = None
  __pump_control_pub__ = None

  # paintTimer = Wait(timerFactory(), 50, singleShot=False)

  def __init__(self, *args, **kwargs) -> None:
    self._debugFlag = False
    self.__pump_control__ = False
    self.pubName = None
    self.topic = None
    self.pub = None
    self.paintTimer = QTimer()
    self.paintTimer.setInterval(50)
    self.paintTimer.setSingleShot(False)
    self.paintTimer.setTimerType(Precise)
    self.pumpTimer = QTimer()
    self.pumpTimer.setInterval(1000)
    self.pumpTimer.timeout.connect(self.pumpControl)
    self.pumpTimer.setSingleShot(False)
    self.pumpTimer.setTimerType(Precise)

    LayoutWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('Welcome to EZRos!')

  def createActionStub(self, ) -> None:
    """Creates a stub file for the action."""
    body = []
    allBases = []
    base = self.__class__
    while base.__bases__:
      allBases.append(base)
      base = base.__bases__[0]
    allBases = [b for b in allBases if issubclass(b, BaseWindow)]
    for base in allBases:
      head = """<br>#  Actions contribution from %s"""
      body.append(head % base.__qualname__)
      for (key, val) in base.getOwnedActions(self).items():
        body.append('%s: QAction' % key)
      head = """<br>#  Namespace contribution from %s"""
      body.append(head % base.__qualname__)
      for (key, val) in base.__dict__.items():
        if not key.startswith('_'):
          body.append('%s: %s' % (key, type(val).__qualname__))
    body = '<br><tab>'.join(body)
    stubFmt = '%s  ' % (os.linesep)
    stub = stubFmt.join([monoSpace(m) for m in [header(), body]])
    here = os.path.dirname(os.path.abspath(__file__))
    name = os.path.splitext(os.path.basename(os.path.abspath(__file__)))
    stubName = ('%s.pyi' % name[0])
    stubPath = os.path.join(here, stubName)
    with open(stubPath, 'w') as f:
      f.write(stub)

  def startPump(self) -> None:
    """LMAO"""
    self.pumpTimer.stop()
    self.pumpTimer.start()

  def connectActions(self) -> None:
    """Connects actions to slots."""
    self.debug01Action.triggered.connect(self.debug01Func)
    self.debug02Action.triggered.connect(self.debug02Func)
    self.debug03Action.triggered.connect(self.debug03Func)
    self.debug04Action.triggered.connect(self.debug04Func)
    self.debug05Action.triggered.connect(self.debug05Func)
    self.debug06Action.triggered.connect(self.debug06Func)
    self.paintTimer.timeout.connect(self.data.update)
    self.paintTimer.start()
    self.topic = Subscriber('/tool/pump_current',
                            Float32Stamped,
                            self.dataCallback)
    self.pubName = '/tool/pump_command'
    self.pub = Publisher(self.pubName, AuxCommand, queue_size=10)

    self._pumpComboBox.addItem('Pump ON')
    self._pumpComboBox.addItem('Pump OFF')

  def updateState(self, *args) -> None:
    """Update state of the main window"""
    sprayText = self._sprayComboBox.currentText()
    pumpText = self._pumpComboBox.currentText()
    # self.state.innerText = '%s, %s' % (pumpText, sprayText)
    # self.state.update()

  def dataCallback(self, data) -> None:
    """Data callback"""
    self.data.callback(data)

  def pumpControl(self, ) -> None:
    """Pump control callback"""
    val = AuxCommand()
    val.for_duration = rospy.Duration.from_sec(1.2)
    val.activate = True if self.toggle.state else False
    self.pub.publish(val)

  def debug01Func(self, ) -> None:
    """Debug01 function"""
    print('Received debug 01 - single click interval')
    if self.pumpTimer.isActive():
      self.pumpTimer.stop()
    else:
      self.pumpTimer.start()

  def debug02Func(self, ) -> None:
    """Debug02 function"""
    print('Received debug 02 - double click interval')
    val = AuxCommand()
    val.for_duration = rospy.Duration.from_sec(0.5)
    val.activate = True
    self.pub.publish(val)

  def debug03Func(self, ) -> None:
    """Debug03 function"""
    print('Received debug 03 - single click')

  def debug04Func(self, ) -> None:
    """Debug04 function"""
    print('Received debug 04 - double click')

  def debug05Func(self, ) -> None:
    """Debug05 function"""
    print('Received debug 05 - triple click')

  def debug06Func(self, ) -> None:
    """Debug06 function"""
    print('Received debug 06 - Force test plot callback')
    self.testPlot(Float64(0.5))
