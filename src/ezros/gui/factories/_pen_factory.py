"""The paint factory creates instances of QBrush, QPen and QFont."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPen
from vistutils.text import stringList
from vistutils.parse import maybe

from ezros.gui.factories import parseColor


def _createPen(*args, **kwargs) -> QPen:
  """Creates a QPen instance."""
  errors = []
  width, color, style = None, None, None
  widthDefault = 1
  colorDefault = QColor(0, 0, 0, 255)
  styleDefault = Qt.PenStyle.SolidLine
  widthKeys = stringList("""width, w, penWidth""")
  colorKeys = stringList("""color, col, penColor, rgb, RGB, rgba, RGBA""")
  styleKeys = stringList("""style, penStyle, lineStyle""")
  widthArg, widthFArg, colorArg, styleArg = None, None, None, None
  widthKwarg, widthFKwarg, colorKwarg, styleKwarg = None, None, None, None
  for arg in args:
    if isinstance(arg, float) and widthFArg is None:
      widthFArg = arg
    elif isinstance(arg, int) and widthArg is None:
      widthArg = arg
    elif isinstance(arg, QColor) and colorArg is None:
      colorArg = arg
    elif isinstance(arg, Qt.PenStyle) and styleArg is None:
      styleArg = arg
  for key in widthKeys:
    if key in kwargs:
      if widthKwarg is None:
        val = kwargs[key]
        if isinstance(val, float):
          widthFKwarg = val
        if isinstance(val, int):
          widthKwarg = val
  for key in colorKeys:
    if key in kwargs:
      if colorKwarg is None:
        val = kwargs[key]
        try:
          colorKwarg = parseColor(val)
        except ValueError as valueError:
          errors.append(valueError)
  for key in styleKeys:
    if key in kwargs:
      if styleKwarg is None:
        val = kwargs[key]
        if isinstance(val, Qt.PenStyle):
          styleKwarg = val
  width = maybe(widthKwarg, widthArg, widthDefault)
  widthF = maybe(widthKwarg, widthArg, None)
  color = maybe(colorKwarg, colorArg, colorDefault)
  style = maybe(styleKwarg, styleArg, styleDefault)
  pen = QPen()
  if widthF is not None:
    if isinstance(widthF, float):
      pen.setWidthF(widthF)
    elif isinstance(widthF, int):
      pen.setWidth(widthF)
  elif width is not None:
    pen.setWidth(width)
  pen.setColor(color)
  pen.setStyle(style)
  return pen


def parsePen(*args, **kwargs) -> QPen:
  """Creates a QPen instance."""
  return _createPen(*args, **kwargs)


def emptyPen() -> QPen:
  """Creates a QPen instance."""
  return _createPen(QColor(0, 0, 0, 0, ), 1, Qt.PenStyle.NoPen)


def textPen() -> QPen:
  """Creates a QPen instance."""
  return _createPen(QColor(0, 0, 0, 255, ), 1, Qt.PenStyle.SolidLine)


def dashPen() -> QPen:
  """Creates a QPen instance."""
  return _createPen(QColor(0, 0, 0, 255, ), 1, Qt.PenStyle.DashLine)


def dotPen() -> QPen:
  """Creates a QPen instance."""
  return _createPen(QColor(0, 0, 0, 255, ), 1, Qt.PenStyle.DotLine)


def dashDotPen() -> QPen:
  """Creates a QPen instance."""
  return _createPen(QColor(0, 0, 0, 255, ), 1, Qt.PenStyle.DashDotLine)
