"""Background paints a background on the widget during the paint hook"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtCore import QRect
from PySide6.QtGui import QPaintEvent, QPainter, QBrush, QPen

from ezros.gui.factories import solidBrush, dashPen, solidPen, emptyBrush
from ezros.gui.factories import emptyPen
from ezros.gui.shortnames import Lavender, LightSteelBlue, DodgerBlue
from ezros.gui.shortnames import DarkCyan, Black
from ezros.gui.widgets import AbstractPaint


class AbstractPaintRect(AbstractPaint):
  """Background paints a background on the widget during the paint hook"""

  def getBrush(self, ) -> QBrush:
    """Returns the brush to paint the background with"""
    widget = self.getWidget()
    if not widget.enabled:
      return solidBrush(Lavender)
    if not widget.underMouse:
      return solidBrush(LightSteelBlue)
    if not widget.buttonDown:
      return solidBrush(DodgerBlue)
    return solidBrush(DarkCyan)

  def getPen(self, ) -> QPen:
    """Returns the pen for the """
    widget = self.getWidget()
    if not widget.enabled:
      return dashPen(Lavender)
    pen = solidPen(Black)
    if not widget.activated and not widget.underMouse:
      pen.setWidth(1)
      return pen
    if widget.underMouse:
      pen.setWidth(2)
    if widget.activated:
      pen.setWidth(3)
    if widget.underMouse and widget.activated:
      pen.setWidth(5)
    return pen

  @abstractmethod
  def getRect(self, event: QPaintEvent, painter: QPainter) -> QRect:
    """Subclasses must implement this method to specify the rect to be
    painted. """

  def applyPaint(self, event: QPaintEvent, painter: QPainter) -> None:
    """Paints the background"""
    AbstractPaint.applyPaint(self, event, painter)
    borderPen = self.getPen()
    fillBrush = self.getBrush()
    blankPen = emptyPen()
    blankBrush = emptyBrush()
    targetRect = self.getRect(event, painter)  # the name, lmao
    painter.setPen(blankPen)
    painter.setBrush(fillBrush)
    painter.drawRect(targetRect)
    painter.setPen(borderPen)
    painter.setBrush(blankBrush)
    painter.drawRect(targetRect)
