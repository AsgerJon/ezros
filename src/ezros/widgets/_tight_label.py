"""TightLabel minimizes its size to fit the text."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QRect, QSize, QPoint
from PySide6.QtGui import QFont, \
  QFontMetrics, \
  QPainter, \
  QPaintEvent, \
  QColor, \
  QBrush, QPen
from attribox import AttriBox
from ezside.core import SolidLine, NoWrap, Black, SolidFill
from ezside.widgets import BaseWidget
from moreutils import StrField
from vistutils.text import monoSpace

from ezros.defaults import Settings
from ezros.rosutils import IntField


class TightLabel(BaseWidget):
  """TightLabel minimizes its size to fit the text."""

  text = StrField('Text')
  fontFamily = StrField('Courier')
  fontSize = IntField(12)
  backgroundColor = QColor(255, 255, 191, 255)
  borderColor = Black
  textColor = Black
  hAlign = StrField('Center')
  vAlign = StrField('Top')

  def getFont(self) -> QFont:
    """Returns the font of the label."""
    font = QFont()
    font.setFamily(self.fontFamily)
    font.setPointSize(self.fontSize)
    return font

  def getFontMetrics(self) -> QFontMetrics:
    """Returns the font metrics of the label."""
    return QFontMetrics(self.getFont())

  def getRect(self) -> QRect:
    """Returns the bounding rect of the label."""
    text = self.text
    return self.getFontMetrics().boundingRect(text)

  def sizeHint(self) -> QSize:
    """Returns the size hint of the label."""
    return self.getRect().size()

  def getLeftIn(self, containingRect: QRect) -> int:
    """Returns the left inset of the label."""
    left0 = Settings.labelLeftMargin
    right0 = Settings.labelRightMargin
    textRect = self.getRect()
    if self.hAlign.lower() == 'left':
      return left0
    elif self.hAlign.lower() == 'right':
      return containingRect.width() - textRect.width() - right0
    elif self.hAlign.lower() == 'center':
      return (containingRect.width() - textRect.width()) // 2
    else:
      e = """Unable to recognize horizontal alignment: '%s'"""
      raise ValueError(monoSpace(e % self.hAlign))

  def getTopIn(self, containingRect: QRect) -> int:
    """Returns the top inset of the label."""
    textRect = self.getRect()
    top0 = Settings.labelTopMargin
    bottom0 = Settings.labelBottomMargin
    if self.vAlign.lower() == 'top':
      return top0
    elif self.vAlign.lower() == 'bottom':
      return containingRect.height() - textRect.height() - bottom0
    elif self.vAlign.lower() == 'center':
      return (containingRect.height() - textRect.height()) // 2
    else:
      e = """Unable to recognize vertical alignment: '%s'"""
      raise ValueError(monoSpace(e % self.vAlign))

  def fitRect(self, containingRect: QRect) -> QRect:
    """Returns the bounding rectangle required for the text placed in
    relation to the target rectangle."""
    self.adjustSize()
    left = self.getLeftIn(containingRect)
    top = self.getTopIn(containingRect)
    leftTop = QPoint(left, top)
    size = self.getRect().size()
    return QRect(leftTop, size)

  def getBackgroundBrush(self) -> QBrush:
    """Returns the background brush of the label."""
    brush = QBrush()
    brush.setColor(QColor(*Settings.labelBackgroundColor))
    brush.setStyle(SolidFill)
    return brush

  def getBorderPen(self) -> QPen:
    """Returns the border pen of the label."""
    pen = QPen()
    pen.setColor(QColor(*Settings.labelBorderColor))
    return pen

  def getTextPen(self) -> QPen:
    """Returns the text pen of the label."""
    pen = QPen()
    pen.setColor(self.textColor)
    pen.setStyle(SolidLine)
    pen.setWidth(1)
    return pen

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the label."""
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    textRect = self.fitRect(viewRect)
    painter.setBrush(self.getBackgroundBrush())
    painter.setPen(self.emptyLine)
    painter.drawRoundedRect(viewRect, 4, 4)
    painter.setPen(self.getBorderPen())
    painter.setBrush(self.emptyBrush)
    painter.drawRoundedRect(viewRect, 4, 4)
    painter.drawText(textRect, self.text)
    painter.end()

  def getText(self) -> str:
    """Getter-function for the text"""
    return self.text

  def setText(self, text: str) -> None:
    """Setter-function for the text"""
    self.text = text
