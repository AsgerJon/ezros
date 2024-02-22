"""The factoids module provides factory functions for creating QBrush,
QPen and QFont instances. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._stub_factory import header
from ._color_factory import colorNames, parseColor
from ._font_factory import parseFont
from ._brush_factory import parseBrush, emptyBrush, solidBrush
from ._pen_factory import parsePen, emptyPen, textPen, dashPen, dotPen
from ._pen_factory import dashDotPen
from ._timer_factory import timerFactory
from ._action_factory import actionFactory
from ._files_factory import filesFactory
from ._edit_factory import editFactory
from ._help_factory import helpFactory
from ._menu_bar_factory import menuBarFactory
