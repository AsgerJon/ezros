"""PenField class for the Pen class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor

from ezros.gui.shortnames import Black


class PenField:
  """PenField class for the Pen class."""

  __field_name__ = None
  __field_owner__ = None
  __field_type__ = None
  __latest_instance__ = None

  __default_line_width__ = None
  __fallback_line_width__ = 1
  __default_color__ = None
  __fallback_color__ = Black
  __default_style__ = None
  __fallback_style__ = Qt.PenStyle.SolidLine

  def _getDefaultLineWidth(self, ) -> int:
    """Return the default line width for the pen."""
    if self.__default_line_width__ is None:
      return self.__fallback_line_width__
    return self.__default_line_width__

  def _getDefaultColor(self, ) -> QColor:
    """Return the default color for the pen."""
    if self.__default_color__ is None:
      return self.__fallback_color__
    return self.__default_color__

  def _getDefaultStyle(self, ) -> Qt.PenStyle:
    """Return the default style for the pen."""
    if self.__default_style__ is None:
      return self.__fallback_style__
    return self.__default_style__

  def _getLineWidthName(self, ) -> str:
    """Return the name of the line width field."""
    pvtName = self.getPrivateName()
    return '_%s_lineWidth__' % pvtName

  def __set_name__(self, owner: type, name: str) -> None:
    """Set the name of the field."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def getPrivateName(self, ) -> str:
    """Return the private name of the field."""
    return '_%s' % self.__field_name__
