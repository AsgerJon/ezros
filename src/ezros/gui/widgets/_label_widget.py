"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QMargins, QRect, QPoint, Qt
from PySide6.QtGui import QPainter, QPaintEvent, QColor, QFont, QFontMetrics

from ezros.gui.factories import parseFont, textPen, emptyBrush, parseBrush
from ezros.gui.shortnames import AlignmentEnum, Black, Yellow, Tight
from ezros.gui.widgets import FillWidget


class LabelWidget(FillWidget):
  """LabelWidget provides a text label."""

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)
    self.setSizePolicy(Tight, Tight)
    self._innerText = None
    self._textColor = None
    self._textFont = None
    self._alignment = None
    strArgs = []
    for arg in args:
      if isinstance(arg, str) and self._innerText is None:
        self.innerText = arg
      elif isinstance(arg, str):
        strArgs.append(arg)
      elif isinstance(arg, QColor) and self._textColor is None:
        self.textColor = arg
      elif isinstance(arg, QFont) and self._textFont is None:
        self.textFont = arg
      elif isinstance(arg, int) and self._alignment is None:
        self.alignment = AlignmentEnum(arg)
    self._textColor = Black if self._textColor is None else self._textColor
    defFont = parseFont(*args, **kwargs)
    self._textFont = defFont if self._textFont is None else self._textFont
    if self._alignment is None:
      self._alignment = AlignmentEnum.LEFT

  def _getAlignment(self) -> AlignmentEnum:
    """Getter for alignment."""
    return self._alignment

  def readyPainter(self, painter: QPainter) -> QPainter:
    """Getter for textFont."""
    painter.setFont(self._textFont)
    painter.setPen(textPen(self._textColor))
    return painter

  def _getFontMetrics(self) -> QFontMetrics:
    """Getter for fontMetrics."""
    return QFontMetrics(self._textFont)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the label."""
    FillWidget.paintEvent(self, event)
    painter = QPainter()
    painter.begin(self)
    painter = self.readyPainter(painter)
    viewRect = painter.viewport()
    borderRect = painter.viewport() - QMargins(2, 2, 2, 2)  # margin
    borderRect.moveCenter(viewRect.center())
    paddedRect = borderRect - QMargins(1, 1, 1, 1)  # border
    paddedRect.moveCenter(viewRect.center())
    innerRect = paddedRect - QMargins(2, 2, 2, 2)  # padding
    innerRect.moveCenter(viewRect.center())
    textRect = self._getFontMetrics().boundingRect(self.innerText)
    textSize = textRect.size()
    textRect = QRect(QPoint(0, 0), textSize)
    align = self._getAlignment()
    textRect = align.apply(textRect, innerRect)
    textBox = textRect + QMargins(1, 1, 1, 1, )
    textBox.moveCenter(textRect.center())
    painter.setBrush(emptyBrush())
    painter.drawRect(paddedRect)
    marker = Yellow
    marker.setAlpha(127)
    painter.setBrush(parseBrush(marker))
    painter.drawRect(textBox)
    painter.setBrush(emptyBrush())
    painter.drawText(textRect, Qt.AlignmentFlag.AlignCenter, self.innerText)
    painter.end()
