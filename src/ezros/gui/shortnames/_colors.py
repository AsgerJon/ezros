"""This file contains named colors for use in the GUI."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor

Black = QColor(0, 0, 0)
White = QColor(255, 255, 255)
Red = QColor(255, 0, 0)
Green = QColor(0, 255, 0)
Blue = QColor(0, 0, 255)

Aqua = QColor(0, 255, 255)
Gray = QColor(128, 128, 128)
Lime = QColor(0, 255, 0)
Maroon = QColor(128, 0, 0)
Navy = QColor(0, 0, 128)
Olive = QColor(128, 128, 0)
Purple = QColor(128, 0, 128)
Silver = QColor(192, 192, 192)
Teal = QColor(0, 128, 128)
Yellow = QColor(255, 255, 0)
Fuchsia = QColor(255, 0, 255)

Orange = QColor(255, 165, 0)
Pink = QColor(255, 192, 203)
Gold = QColor(255, 215, 0)
Cyan = QColor(0, 255, 255)
Magenta = QColor(255, 0, 255)
Indigo = QColor(75, 0, 130)
Coral = QColor(255, 127, 80)
Chocolate = QColor(210, 105, 30)
Tomato = QColor(255, 99, 71)
Lavender = QColor(230, 230, 250)


def colorDict() -> dict[str, QColor]:
  """Returns a dictionary of named colors"""
  return {
    'Black'    : Black, 'White': White, 'Red': Red, 'Green': Green,
    'Blue'     : Blue,
    'Aqua'     : Aqua, 'Gray': Gray, 'Lime': Lime, 'Maroon': Maroon,
    'Navy'     : Navy,
    'Olive'    : Olive, 'Purple': Purple, 'Silver': Silver, 'Teal': Teal,
    'Yellow'   : Yellow,
    'Fuchsia'  : Fuchsia, 'Orange': Orange, 'Pink': Pink, 'Gold': Gold,
    'Cyan'     : Cyan,
    'Magenta'  : Magenta, 'Indigo': Indigo, 'Coral': Coral,
    'Chocolate': Chocolate, 'Tomato': Tomato, 'Lavender': Lavender
  }


__all__ = [
  'Black', 'White', 'Red', 'Green', 'Blue',
  'Aqua', 'Gray', 'Lime', 'Maroon', 'Navy',
  'Olive', 'Purple', 'Silver', 'Teal', 'Yellow',
  'Fuchsia', 'Orange', 'Pink', 'Gold', 'Cyan',
  'Magenta', 'Indigo', 'Coral', 'Chocolate', 'Tomato', 'Lavender'
]
