"""The parseHex function creates a QColor representation of the hex valued
color given. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QColor
from vistutils.text import monoSpace


def _parseByte(*chars) -> int:
  """This method receives one or two hex digits and returns the integer
  value."""
  digs = []
  for arg in chars:
    if isinstance(arg, int):
      arg = '%d' % arg
    if isinstance(arg, str):
      for char in arg:
        digs.append(char.upper())
  digs.reverse()
  hexVals = '0123456789ABCDEF'
  hexDict = {k: v for (k, v) in zip(hexVals, range(16))}
  for arg in digs:
    if arg not in hexVals:
      e = """Expected only hex values, but received: '%s'!""" % arg
      raise ValueError(e)
  out = 0
  for (i, c) in enumerate(digs):
    out += (hexDict[c] * (16 ** i))
  return out


def _splitHex(hexColor: str) -> list:
  """Splits the string to pairs of hex values"""
  out = []
  hexChars = None
  if hexColor[0] == '#':
    return _splitHex(hexColor[1:])
  if len(hexColor) % 2:
    e = """Expected even number of characters but received: '%d'"""
    raise ValueError(monoSpace(e % len(hexColor)))
  if len(hexColor) == 6:
    return _splitHex('%sFF' % hexColor)
  if len(hexColor) == 8:
    hexChars = [i for i in hexColor]
  if hexChars is None:
    raise ValueError
  while hexChars:
    channel = str(hexChars.pop(0))
    out.append('%s%s' % (channel, hexChars.pop(0)))
  return out


def parseHex(hexColor: Any) -> QColor:
  """Parses hex color"""
  if isinstance(hexColor, QColor):
    return hexColor
  if not isinstance(hexColor, str):
    raise TypeError
  try:
    hexChars = _splitHex(hexColor)
  except ValueError as valueError:
    color = QColor(hexColor)
    if color is None:
      raise valueError
    return color
  rgba = [_parseByte(chars) for chars in hexChars]
  return QColor(*rgba)
