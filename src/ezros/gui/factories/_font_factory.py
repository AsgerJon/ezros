"""The fontFactory function creates instances of QFont."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QFont
from vistutils.text import stringList
from vistutils.parse import maybe


def _createFont(*args, **kwargs) -> QFont:
  """Creates a QFont instance."""
  familyDefault = "Courier"
  sizeDefault = 16
  weightDefault = QFont.Weight.Normal
  familyArg, sizeArg, weightArg = None, None, None
  familyKeys = stringList("""family, fontFamily""")
  sizeKeys = stringList("""size, fontSize""")
  weightKeys = stringList("""weight, fontWeight""")
  for arg in args:
    if isinstance(arg, str) and familyArg is None:
      familyArg = arg
    elif isinstance(arg, int) and sizeArg is None:
      sizeArg = arg
  familyKwarg, sizeKwarg, weightKwarg = None, None, None
  for key in familyKeys:
    if key in kwargs and familyKwarg is None:
      familyKwarg = kwargs[key]
  for key in sizeKeys:
    if key in kwargs and sizeKwarg is None:
      sizeKwarg = kwargs[key]
  for key in weightKeys:
    if key in kwargs and weightKwarg is None:
      weightKwarg = kwargs[key]
  family = maybe(familyKwarg, familyArg, familyDefault)
  size = maybe(sizeKwarg, sizeArg, sizeDefault)
  weight = maybe(weightKwarg, weightArg, weightDefault)
  font = QFont()
  font.setFamily(family)
  font.setPointSize(size)
  font.setWeight(weight)
  return font


def parseFont(*args, **kwargs) -> QFont:
  """Parses a QFont instance."""
  return _createFont(*args, **kwargs)
