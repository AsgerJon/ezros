"""The Talker class publishes sample data to topics."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.core import LawnGreen
from ezside.windows import BaseWindow
from ezside.widgets import BaseWidget
from ezside.widgets import CornerPanel, VerticalPanel, HorizontalPanel

from ezros.widgets import TightLabel, Grid, StrInput


class TalkWindow(BaseWindow):
  """Talker class publishes sample data to topics."""

  topicNameWidget = AttriBox[StrInput]('Topic Name',
                                       'Submit',
                                       '[Not set!]',
                                       'Enter topic name here')

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[Grid]()

  left = AttriBox[VerticalPanel](LawnGreen)
  top = AttriBox[HorizontalPanel](LawnGreen)
  right = AttriBox[VerticalPanel](LawnGreen)
  bottom = AttriBox[HorizontalPanel](LawnGreen)
  topLeft = AttriBox[CornerPanel](LawnGreen)
  topRight = AttriBox[CornerPanel](LawnGreen)
  bottomLeft = AttriBox[CornerPanel](LawnGreen)
  bottomRight = AttriBox[CornerPanel](LawnGreen)

  def initUi(self) -> None:
    """The initUi method initializes the user interface of the window."""
    self.setMinimumSize(400, 400)
    rowCount = 1
    colCount = 1

    self.baseLayout.addWidget(self.bottomRight, rowCount + 1, 1 + colCount)
    self.baseLayout.addWidget(self.bottom, rowCount + 1, 1, 1, colCount)
    self.baseLayout.addWidget(self.bottomLeft, rowCount + 1, 0)

    self.baseLayout.addWidget(self.left, 1, 0, rowCount, 1)
    self.baseLayout.addWidget(self.right, 1, colCount + 1, rowCount, 1)

    self.baseLayout.addWidget(self.topLeft, 0, 0)
    self.baseLayout.addWidget(self.top, 0, 1, 1, colCount)
    self.baseLayout.addWidget(self.topRight, 0, colCount + 1)

    self.baseLayout.addWidget(self.topicNameWidget, 1, 1, 1, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  def initActions(self) -> None:
    """The initActions method initializes the actions of the window."""
    pass
