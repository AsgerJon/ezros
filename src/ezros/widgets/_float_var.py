"""StrVar provides a widget for setting a float variable."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.widgets import BaseWidget

from ezros.widgets import Horizontal, Label, HorizontalSpacer


class FloatVar(BaseWidget):
  """StrVar provides a widget for setting a float variable."""

  baseLayout = AttriBox[Horizontal]()
  label = AttriBox[Label]()
  slider = AttriBox[HSlider]()
  button = AttriBox[QPushButton]()
  h1 = AttriBox[HorizontalSpacer]()

  clicked = Signal()

  valueChangedFull = Signal(str, str)
  valueChangedNew = Signal(str)
  valueChangedOld = Signal(str)
  valueChanged = Signal()

  valueRequestedFull = Signal(str)
  valueRequested = Signal()

  valueClearedFull = Signal(str)
  valueCleared = Signal()
