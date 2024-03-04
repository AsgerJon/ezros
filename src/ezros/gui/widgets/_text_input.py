"""The TextInput widget provides for user input."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.gui.factories import parseFont, textPen
from ezros.gui.paint import FillRect, BorderRect
from ezros.gui.shortnames import Black, Silver
from ezros.gui.widgets import PaintWidget, BaseWidget, BaseLayout, \
  BaseLineEdit
from _dep.morevistutils import TextField, Wait


class TextInput(PaintWidget):
  """The TextInput class provides for user input."""

  __fallback_place_holder__ = 'Enter text here...'
  __place_holder_text__ = None

  innerText = TextField('lmao')
  textFont = Wait(parseFont, 'Arial', 12)
  textLine = Wait(textPen, Black)
  placeHolderFont = Wait(parseFont, 'Arial', 10)
  placeHolderLine = Wait(textPen, Silver)

  outerFill = FillRect()
  outerBorder = BorderRect()
  innerFill = FillRect()
  innerBorder = BorderRect()

  baseWidget = Wait(BaseWidget)
  baseLayout = Wait(BaseLayout)
  baseLine = Wait(BaseLineEdit)

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    self.__place_holder_text__ = TextField.parseText(
      *args, self.__fallback_place_holder__)
