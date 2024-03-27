"""This module provides the specialized widgets used by the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._timer import Timer
from ._abstract_command_widget import AbstractCommandWidget
from ._parse_text import parseText
from ._layouts import Grid, Vertical, Horizontal
from ._push_button import PushButton
from ._line_edit import LineEdit
from ._command_control import CommandControl
from ._live_view import LiveView
from ._tight_label import TightLabel
from ._spin_box import SpinBox
from ._ros_talker import RosTalker
from ._str_input import StrInput
from ._dyn_chart import DynChart
from ._tab_page import TabPage
from ._tab_widget import TabWidget
from ._button_old import Button
from ._static_chart import StaticChart
