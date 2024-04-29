"""ControlPeriodic provides a widget with two sliders controlling the on
and off periods of a periodic signal."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from attribox import AttriBox
from ezside.widgets import Horizontal


class ControlPeriodic:
  """ControlPeriodic provides a widget with two sliders controlling the on
  and off periods of a periodic signal."""

  baseLayout = AttriBox[Horizontal]()
