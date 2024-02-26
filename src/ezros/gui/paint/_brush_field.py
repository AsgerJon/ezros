"""BrushField provides a descriptor class for the QBrush."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QBrush
from vistutils.waitaminute import typeMsg

from ezros.gui.factories import parseColor
from ezros.gui.paint import ColorField
from ezros.gui.shortnames import Lime
from morevistutils.fields import AbstractBaseField


class BrushField(AbstractBaseField):
  """BrushField provides a descriptor class for the QBrush."""

  __default_solid_color__ = None
  __fallback_solid_color__ = Lime

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the BrushField instance."""
    AbstractBaseField.__init__(self, *args, **kwargs)
    color = parseColor(*args, **kwargs, strict=False)
    if color is not None:
      if isinstance(color, QColor):
        self._setDefaultSolidColor(color)
      else:
        e = typeMsg('color', color, QColor)
        raise TypeError(e)

  def _setDefaultSolidColor(self, color: QColor) -> None:
    """Set the default solid color for the brush."""
    self.__default_solid_color__ = color

  def _getDefaultSolidColor(self, ) -> QColor:
    """Return the default solid color for the brush."""
    if self.__default_solid_color__ is None:
      return self.__fallback_solid_color__
    if isinstance(self.__default_solid_color__, QColor):
      return self.__default_solid_color__
    e = typeMsg('color', self.__default_solid_color__, QColor)
    raise TypeError(e)

  def _getSolidColorName(self, ) -> str:
    """Return the name of the solid color field."""
    pvtName = self.getPrivateName()
    return '_%s_solidColor__' % pvtName

  def _getSolidColor(self, instance: Any, owner: type, **kwargs) -> QColor:
    """Return the solid color for the brush."""
    pvtName = self._getSolidColorName()
    if getattr(instance, pvtName, None) is not None:
      color = getattr(instance, pvtName)
      if isinstance(color, QColor):
        return color
      e = typeMsg('color', color, QColor)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self._getDefaultSolidColor())
    return self._getSolidColor(instance, owner, _recursion=True, **kwargs)

  def instantiate(self, instance: Any, owner: type) -> None:
    """Instantiate the field."""
    pvtName = self.getPrivateName()
    brushStyle = Qt.BrushStyle.SolidPattern
    color = self._getSolidColor(instance, owner)
    brush = QBrush()
    brush.setColor(color)
    brush.setStyle(brushStyle)
    if isinstance(brush, QBrush):
      return setattr(instance, pvtName, brush)
    eColor, eStyle = None, None
    if not isinstance(color, QColor):
      eColor = typeMsg('color', color, QColor)
    if not isinstance(brushStyle, Qt.BrushStyle):
      eStyle = typeMsg('brushStyle', brushStyle, Qt.BrushStyle)
    e = typeMsg('brush', brush, QBrush)
    if eColor is None and eStyle is None:
      raise TypeError(e)
    if eColor is None:
      raise TypeError(e) from TypeError(eStyle)
    if eStyle is None:
      raise TypeError(e) from TypeError(eColor)
    try:
      raise TypeError(eColor)
    except TypeError as colorTypeError:
      try:
        raise TypeError(eStyle) from colorTypeError
      except TypeError as styleTypeError:
        raise TypeError(e) from styleTypeError

  def __get__(self, instance: Any, owner: type, **kwargs) -> QBrush:
    """Getter function for the field"""
    pvtName = self.getPrivateName()
    if getattr(instance, pvtName, None) is not None:
      brush = getattr(instance, pvtName)
      if isinstance(brush, QBrush):
        return brush
      e = typeMsg('brush', brush, QBrush)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.instantiate(instance, owner)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: QBrush) -> Never:
    """The BrushField is read-only as it reflects underlying primitives
    such as color and style."""
    msg = self.__class__.__set__.__doc__
    raise TypeError(msg)

  def __delete__(self, instance: Any) -> Never:
    """Illegal deleter function."""
    msg = self.__class__.__set__.__doc__
    raise TypeError(msg)
