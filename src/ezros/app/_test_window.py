"""TestWindow lmao"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside import BaseWindow
from ezside.widgets import Vertical, BaseWidget, EntryForm, Horizontal

from ezros.mathview import Canvas


class TestWindow(BaseWindow):
  """yolo"""

  baseLayout = AttriBox[Vertical]()
  domainLayout = AttriBox[Horizontal]()
  baseWidget = AttriBox[BaseWidget]()
  domainWidget = AttriBox[BaseWidget]()
  mapping = AttriBox[EntryForm]('Mapping Function: ')
  xMin = AttriBox[EntryForm]('xMin', Vertical)
  xMax = AttriBox[EntryForm]('xMax', Vertical)
  canvas = AttriBox[Canvas]()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.mapping.title = 'mapping'
    self.xMin.title = 'xMin'
    self.xMax.title = 'xMax'
    self.mapping.hSpacer.setDebugFlag(False)
    self.mapping.vSpacer.setDebugFlag(False)
    self.xMin.hSpacer.setDebugFlag(False)
    self.xMin.vSpacer.setDebugFlag(False)
    self.xMax.hSpacer.setDebugFlag(False)
    self.xMax.vSpacer.setDebugFlag(False)
    self.domainLayout.addWidget(self.xMin)
    self.domainLayout.addWidget(self.xMax)
    self.domainWidget.setLayout(self.domainLayout)
    self.baseLayout.addWidget(self.canvas)
    self.baseLayout.addWidget(self.mapping)
    self.baseLayout.addWidget(self.domainWidget)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  def initActions(self) -> None:
    """Initialize the actions."""
    self.mapping.newText.connect(self.canvas.setMapping)
    self.xMin.newText.connect(self.updateDomain)
    self.xMax.newText.connect(self.updateDomain)

  def updateDomain(self, *_) -> None:
    """Update the domain of the mapping."""
    self.canvas.setDomain(float(self.xMin.value), float(self.xMax.value))
    self.canvas.update()
