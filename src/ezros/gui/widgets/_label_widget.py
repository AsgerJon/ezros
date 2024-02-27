"""LabelWidget provides a text label."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from ezros.gui.paint import BorderRect, FillRect, TextRect
from ezros.gui.widgets import PaintWidget


class LabelWidget(PaintWidget):
  """LabelWidget provides a text label."""

  __inner_text__ = None

  innerText = Field()
  outerFill = FillRect()
  outerLine = BorderRect()
  innerFill = FillRect()
  innerLine = BorderRect()
  textPaint = TextRect()

  @innerText.GET
  def _getInnerText(self, **kwargs) -> str:
    if self.__inner_text__ is None:
      if __name__ == '__main__':
        if kwargs.get('recursion', False):
          raise RecursionError
        self.__inner_text__ = kwargs['innerText']
        return self._getInnerText(recursion=True)
    return self.__inner_text__

  @innerText.SET
  def _setInnerText(self, value: str) -> None:
    if isinstance(value, str):
      self.__inner_text__ = value
    else:
      e = typeMsg('inner_text', value, str)
      raise TypeError(e)

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    self.innerText = [*[arg for arg in args if isinstance(arg, str)], ''][0]

  def paintHook(self, event: QPaintEvent, painter: QPainter) -> None:
    """Hook for painting the widget."""
    self.outerFill.paintOp(event, painter)
    self.outerLine.paintOp(event, painter)
    self.innerFill.paintOp(event, painter)
    self.innerLine.paintOp(event, painter)
    self.textPaint.paintOp(event, painter, self.innerText)
