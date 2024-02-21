"""LayoutWindow class for the layout window of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.widgets import LabelWidget, BaseWidget, BaseLayout
from ezros.gui.windows import BaseWindow
from morevistutils.fields import Field


class _LayoutWindowFields(BaseWindow):
  """Fields for the LayoutWindow class."""

  baseWidget = Field(BaseWidget, )
  baseLayout = Field(BaseLayout, )
  welcomeLabel = Field(LabelWidget, 'Welcome', )
  goodbyeLabel = Field(LabelWidget, 'Goodbye', )


class LayoutWindow(_LayoutWindowFields):
  """LayoutWindow class for the layout window of the application."""

  def __init__(self, *args, **kwargs) -> None:
    _LayoutWindowFields.__init__(self, *args, **kwargs)
    self.setMinimumSize(480, 320)

  def initUI(self) -> None:
    """Sets up the widgets"""
    _LayoutWindowFields.initUI(self)
    self.baseLayout.addWidget(self.welcomeLabel, 0, 0)
    self.baseLayout.addWidget(self.goodbyeLabel, 1, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
