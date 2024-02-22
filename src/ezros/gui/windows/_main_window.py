"""MainWindow provides the main application window. The BaseWindow class
provides menus and actions. The Layout Window class provides the layout of
widget that appear on the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QMessageBox
from icecream import ic
from vistutils.text import monoSpace

from ezros.gui.factories import header
from ezros.gui.windows import LayoutWindow

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """MainWindow provides the main application window. The BaseWindow class
  provides menus and actions. The Layout Window class provides the layout of
  widget that appear on the main application window"""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('Welcome to EZRos!')

  def showActions(self) -> None:
    """Shows the actions for the window."""
    actionTxt = []
    for action in self.actions():
      actionTxt.append(action.text())
    QMessageBox.about(self, 'Actions', '\n'.join(actionTxt))

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Handles the mouse release event."""
    self.showActions()

  def createActionStub(self, ) -> None:
    """Creates a stub file for the action."""
    actions = self._getOwnedActions()
    body = ['#  Defined Actions:']
    for (key, val) in actions.items():
      body.append('%s: QAction' % key)
    allBases = []
    base = self.__class__
    while base.__bases__:
      allBases.append(base)
      base = base.__bases__[0]
    for base in allBases:
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
