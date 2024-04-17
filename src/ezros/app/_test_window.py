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
