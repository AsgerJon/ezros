"""PromptWidget """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from ezros.gui.widgets import LabelWidget, MouseAware

ic.configureOutput(includeContext=True)


class PromptWidget(LabelWidget, MouseAware):
  """PromptWidget subclasses the LabelWidget class allowing for user input
  to be placed on the label and used as part of the general GUI
  functionality. """
