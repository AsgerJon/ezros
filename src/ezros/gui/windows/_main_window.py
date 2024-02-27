"""MainWindow provides the main application window. The BaseWindow class
provides menus and actions. The Layout Window class provides the layout of
widget that appear on the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from time import sleep
from typing import Any
import os

import rospy
from PySide6.QtCore import Signal
from rospy import Subscriber, init_node
from std_msgs.msg import Float64
from PySide6.QtWidgets import QMessageBox
from icecream import ic
from vistutils.text import monoSpace

from ezros.gui.factories import header, timerFactory
from ezros.gui.windows import LayoutWindow, BaseWindow
from ezros.rosutils import getNodeStatus
from morevistutils import Wait

os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """MainWindow provides the main application window. The BaseWindow class
  provides menus and actions. The Layout Window class provides the layout of
  widget that appear on the main application window"""

  subscribe = Signal(Any)

  paintTimer = Wait(timerFactory(), 50, singleShot=False)

  def __init__(self, *args, **kwargs) -> None:
    self._debugFlag = False
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

  def connectActions(self) -> None:
    """Connects actions to slots."""
    self.debug01Action.triggered.connect(self.debug01Func)
    self.debug02Action.triggered.connect(self.debug02Func)
    self.debug03Action.triggered.connect(self.debug03Func)
    self.debug04Action.triggered.connect(self.debug04Func)
    self.debug05Action.triggered.connect(self.debug05Func)
    self.debug06Action.triggered.connect(self.debug06Func)
    self.subscribe.connect(self.data.callback)
    self.paintTimer.timeout.connect(self.testPaint)
    self._pumpComboBox.addItem('Pump Idle')
    self._pumpComboBox.addItem('Pump ON')
    self._pumpComboBox.currentIndexChanged.connect(self.updateState)
    self._sprayComboBox.addItem('Spray Idle')
    self._sprayComboBox.addItem('Spray ON')
    self._sprayComboBox.currentIndexChanged.connect(self.updateState)

  def updateState(self, *args) -> None:
    """Update state of the main window"""
    sprayText = self._sprayComboBox.currentText()
    pumpText = self._pumpComboBox.currentText()
    self.state.innerText = '%s, %s' % (pumpText, sprayText)
    self.state.update()

  def testPaint(self) -> None:
    """Test paint method"""
    self.plot.update()
    self.data.update()

  def testPlot(self, data: Any) -> None:
    """Test plot method"""
    self.plot.append(data.data)

  def debug01Func(self, ) -> None:
    """Debug01 function"""
    print('Received debug 01 - Starting test')
    nodeName = 'Subscriber'
    init_node(nodeName, anonymous=False)
    self.subscribe.connect(self.data.callback)
    Subscriber('topic', Float64, self.subscribe.emit)
    self.paintTimer.start()

  def debug02Func(self, ) -> None:
    """Debug02 function"""
    print('Received debug 02 - setting debug flag')
    ic(self.paintTimer.isActive())
    self._debugFlag = True

  def debug03Func(self, ) -> None:
    """Debug03 function"""
    print('Received debug 03 - updating plot')
    self.plot.update()

  def debug04Func(self, ) -> None:
    """Debug04 function"""
    print('Received debug 04 - start timer')
    ic(self.paintTimer)
    self.paintTimer.start()

  def debug05Func(self, ) -> None:
    """Debug05 function"""
    print('Received debug 05 - force repaint')
    self.plot.update()

  def debug06Func(self, ) -> None:
    """Debug06 function"""
    print('Received debug 06 - Force test plot callback')
    self.testPlot(Float64(0.5))
