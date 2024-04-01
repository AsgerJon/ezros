"""Label prints centered text"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt, QRect, QRectF
from PySide6.QtGui import QPainter, \
  QPen, \
  QBrush, \
  QFont, \
  QPaintEvent, \
  QFontMetrics
from ezside.core import SolidLine, SolidFill, emptyPen, emptyBrush
from ezside.moreutils import StrField
from ezside.widgets import BaseWidget
from icecream import ic
from vistutils.waitaminute import typeMsg

from ezros.defaults import Settings
from ezros.rosutils import EmptyField

ic.configureOutput(includeContext=True, )


class Label(BaseWidget):
  """Label prints centered text"""

  __inner_text__ = None

  text = EmptyField()

  @text.GET
  def getText(self) -> str:
    """Get the text."""
    return self.__inner_text__

  @text.SET
  def setText(self, text: str) -> None:
    """Set the text."""
    self.__inner_text__ = text
    rect = self.getTextRect()
    self.setMinimumSize(rect.size())
    ic('cunt')

  @text.DEL
  def delText(self) -> None:
    """Delete the text."""
    self.__inner_text__ = None

  @staticmethod
  def getTextPen() -> QPen:
    """Returns the pen of the label."""
    pen = QPen()
    pen.setColor(Settings.getLabelTextColor())
    return pen

  @staticmethod
  def getBorderPen() -> QPen:
    """Returns the border pen of the label."""
    pen = QPen()
    pen.setColor(Settings.getLabelBorderColor())
    pen.setWidth(Settings.labelBorderWidth)
    pen.setStyle(SolidLine)
    return pen

  @staticmethod
  def getBackgroundBrush() -> QBrush:
    """Returns the background brush of the label."""
    brush = QBrush()
    brush.setStyle(SolidFill)
    brush.setColor(Settings.getLabelBackgroundColor())
    return brush

  @staticmethod
  def getTextFont() -> QFont:
    """Returns the font of the label."""
    font = QFont()
    font.setFamily(Settings.fontFamily)
    font.setPointSize(Settings.labelFontSize)
    return font

  @classmethod
  def getFontMetrics(cls) -> QFontMetrics:
    """Returns the font metrics of the label."""
    return QFontMetrics(cls.getTextFont())

  def getTextRect(self) -> QRect:
    """Returns the bounding rect of the label."""
    margins = Settings.getLabelMargins()
    return self.getFontMetrics().boundingRect(self.text) + margins

  def initUi(self) -> None:
    """Initialize the user interface."""
    BaseWidget.initUi(self)
    self.setMinimumSize(self.getTextRect().size())

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint the label."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.Antialiasing)
    textRect = self.getTextRect()
    viewRect = painter.viewport()
    textRect.moveCenter(viewRect.center())
    painter.setBrush(self.getBackgroundBrush())
    painter.setPen(emptyPen())
    painter.drawRect(viewRect)
    painter.setPen(self.getBorderPen())
    painter.setBrush(emptyBrush())
    painter.drawRect(viewRect)
    painter.setPen(self.getTextPen())
    painter.setBrush(emptyBrush())
    if not isinstance(textRect, (QRect, QRectF)):
      e = typeMsg('textRect', textRect, QRect)
      raise TypeError(e)
    if not isinstance(self.text, str):
      e = typeMsg('self.text', self.text, str)
      raise TypeError(e)
    painter.drawText(textRect, Qt.AlignCenter, self.text)
    painter.end()