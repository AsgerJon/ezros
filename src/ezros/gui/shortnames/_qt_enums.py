"""Common names from Qt."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QSizePolicy

#  Timer types
Precise = Qt.TimerType.PreciseTimer
Coarse = Qt.TimerType.CoarseTimer
VeryCoarse = Qt.TimerType.VeryCoarseTimer
#  Alignments
Left = Qt.AlignmentFlag.AlignLeft
Right = Qt.AlignmentFlag.AlignRight
Top = Qt.AlignmentFlag.AlignTop
Bottom = Qt.AlignmentFlag.AlignBottom
Center = Qt.AlignmentFlag.AlignCenter
HCenter = Qt.AlignmentFlag.AlignHCenter
VCenter = Qt.AlignmentFlag.AlignVCenter
#  Pen styles
SolidLine = Qt.PenStyle.SolidLine
DashLine = Qt.PenStyle.DashLine
DotLine = Qt.PenStyle.DotLine
DashDotLine = Qt.PenStyle.DashDotLine
DashDotDotLine = Qt.PenStyle.DashDotDotLine
EmptyLine = Qt.PenStyle.NoPen
#  Brush styles
SolidFill = Qt.BrushStyle.SolidPattern
#  Size policy
Spread = QSizePolicy.Policy.MinimumExpanding
Fixed = QSizePolicy.Policy.Fixed
Tight = QSizePolicy.Policy.Maximum

__all__ = ['Precise', 'Coarse', 'VeryCoarse', 'Left', 'Right', 'Top',
           'Bottom', 'Center', 'HCenter', 'VCenter', 'SolidLine', 'DashLine',
           'DotLine', 'DashDotLine', 'DashDotDotLine', 'EmptyLine',
           'SolidFill', 'Spread', 'Fixed', 'Tight']
