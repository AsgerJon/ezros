"""Tester class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QPainter, QPaintEvent
from vistside.core import parsePen, Black
from vistside.widgets import BaseWidget, BaseLayoutField, LabelWidget, \
  LabelField
from vistutils.fields import Wait


class TestWidget(BaseWidget):
  """The TestWidget class is a widget for testing purposes."""

  baseLayout = BaseLayoutField(layout='vertical')
  banner = LabelField('Yolo')

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestWidget."""
    BaseWidget.__init__(self, *args, **kwargs)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.banner)
    self.setLayout(self.baseLayout)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setBrush(self.emptyBrush)
    painter.setPen(parsePen(Black, 10))
    viewRect = painter.viewport()
    viewRect.adjust(10, 10, -10, -10)
    painter.drawRect(viewRect)
    painter.end()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> TestWidget:
    """Returns the default value for the field."""
    return cls().apply((args, kwargs))

  def apply(self, value: Any) -> TestWidget:
    """Applies the arguments contained in value to the widget."""
    return self


class TestField(Wait):
  """The TestField class is a field for testing purposes."""

  def __init__(self, *args, **kwargs) -> None:
    """Create a new TestField."""
    Wait.__init__(self, TestWidget, *args, **kwargs)
