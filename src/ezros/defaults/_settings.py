"""The defaults class provides a base class for all defaults classes in the
application. During development values are placed in the class body. Once
deployed values that should be configurable by the user are loaded from a
file. When the users make defaults changes, this file is updated."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtCore import QMargins
from PySide6.QtGui import QFont, QColor


class Settings:
  """The defaults class provides a base class for all defaults classes in the
  application. During development values are placed in the class body. Once
  deployed values that should be configurable by the user are loaded from a
  file. When the users make defaults changes, this file is updated."""

  @staticmethod
  def getButtonStyle() -> str:
    """Get the button style."""
    cornerRadius = 4
    borderWidth = 2
    borderColor = '#000000'
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, '_button_style.qss')) as file:
      print(file.read())
      return file.read()

  labelBackgroundColor = (255, 255, 192, 255)
  labelBorderColor = (0, 0, 0, 255)
  labelTextColor = (0, 0, 0, 255)
  labelBorderWidth = 2
  labelTopMargin = 2
  labelBottomMargin = 2
  labelLeftMargin = 32
  labelRightMargin = 32

  fontFamily = 'Montserrat'
  buttonFontSize = 12
  labelFontSize = 14
  headerFontSize = 16

  numPoints = 256

  spacerVisibility = False

  @classmethod
  def getButtonFont(cls) -> QFont:
    """Get the button font."""
    font = QFont()
    font.setFamily(cls.fontFamily)
    font.setPointSize(cls.buttonFontSize)
    return font

  @classmethod
  def getLabelMargins(cls, ) -> QMargins:
    """Returns the margins of the label."""
    return QMargins(cls.labelLeftMargin,
                    cls.labelTopMargin,
                    cls.labelRightMargin,
                    cls.labelBottomMargin, )

  @classmethod
  def getHeaderFont(cls) -> QFont:
    """Get the header font."""
    font = QFont()
    font.setFamily(cls.fontFamily)
    font.setPointSize(cls.headerFontSize)
    return font

  @classmethod
  def getLabelBackgroundColor(cls) -> QColor:
    """Get the label background color."""
    return QColor(*cls.labelBackgroundColor)

  @classmethod
  def getLabelBorderColor(cls) -> QColor:
    """Get the label border color."""
    return QColor(*cls.labelBorderColor)

  @classmethod
  def getLabelTextColor(cls) -> QColor:
    """Get the label text color."""
    return QColor(*cls.labelTextColor)
