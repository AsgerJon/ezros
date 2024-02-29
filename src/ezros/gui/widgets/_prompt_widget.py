"""PromptWidget """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, QEvent, Slot
from PySide6.QtGui import QMouseEvent
from icecream import ic

from ezros.gui.factories import timerFactory
from ezros.gui.shortnames import MouseBtn, LeftBtn, RightBtn, \
  MiddleBtn, NextBtn, BackBtn
from ezros.gui.widgets import LabelWidget, MouseAware
from morevistutils import Wait, Field

ic.configureOutput(includeContext=True)


class PromptWidget(LabelWidget, MouseAware):
  """PromptWidget subclasses the LabelWidget class allowing for user input
  to be placed on the label and used as part of the general GUI
  functionality. """
