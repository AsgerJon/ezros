"""BoolProg provides a subclass of BoolPub that turns a signal on and off
according to a specified program. This program is defined by an on period
and an off period that repeats."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.rosutils import BoolPub


class BoolProg(BoolPub):
  """BoolProg provides a subclass of BoolPub that turns a signal on and off
  according to a specified program. This program is defined by an on period
  and an off period that repeats."""

  __high_time__ = None
  __low_time__ = None
  __high_fallback__ = 1
  __low_fallback__ = 1

  lowTimer = AttriBox[QTimer]()
  highTimer = AttriBox[QTimer]()
