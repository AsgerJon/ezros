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
from ezside.core import SolidLine, Black, SolidFill
from ezside.widgets import BaseWidget
from ezside.moreutils import StrField
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

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
  vAlign = StrField('Center')

  def __init__(self, text: str = None) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, )
    if text is not None:
      self.text = str(text)

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
    return self.getFontMetrics().boundingRect(self.text)

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

  def alignRect(self,
                movingRect: QRect | QSize,
                containingRect: QRect) -> QRect:
    """Returns the aligned rectangle."""
    if isinstance(movingRect, QRect):
      movingRect = movingRect.size()
    if self.hAlign.lower() == 'center':
      left = (containingRect.width() - movingRect.width()) // 2
    elif self.hAlign.lower() == 'left':
      left = containingRect.left()
    elif self.hAlign.lower() == 'right':
      left = containingRect.right() - movingRect.width()
    else:
      e = """Unable to recognize horizontal alignment: '%s'"""
      raise ValueError(monoSpace(e % self.hAlign))
    if self.vAlign.lower() == 'center':
      top = (containingRect.height() - movingRect.height()) // 2
    elif self.vAlign.lower() == 'top':
      top = containingRect.top()
    elif self.vAlign.lower() == 'bottom':
      top = containingRect.bottom() - movingRect.height()
    else:
      e = """Unable to recognize vertical alignment: '%s'"""
      raise ValueError(monoSpace(e % self.vAlign))
    leftTop = QPoint(left, top)
    return QRect(leftTop, movingRect)

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
    try:
      painter = QPainter()
      painter.begin(self)
      viewRect = painter.viewport()
      textRect = self.getRect()
      marginedSize = (textRect + Settings.getLabelMargins()).size()
      marginedRect = self.alignRect(marginedSize, viewRect)
      textRect.moveCenter(marginedRect.center())
      painter.setBrush(self.getBackgroundBrush())
      painter.setPen(self.emptyLine)
      painter.drawRoundedRect(marginedRect, 4, 4)
      painter.setPen(self.getBorderPen())
      painter.setBrush(self.emptyBrush)
      painter.drawRoundedRect(marginedRect, 4, 4)
      painter.drawText(textRect, self.text)
      painter.end()
    except Exception as e:
      print(e)
      raise SystemExit from e

  def getText(self) -> str:
    """Getter-function for the text"""
    return self.text

  def setText(self, text: str) -> None:
    """Setter-function for the text"""
    self.text = text
