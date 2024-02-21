"""Parses a string to a float"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import numpy as np


def parseFloat(floatString: str) -> float:
  """Parses a string to a float"""
  floatString = ''.join([c for c in floatString if c in '0123456789+-.'])
  if floatString == '---':
    return np.nan
  try:
    # Try to convert the string to a float directly
    return float(floatString)
  except ValueError as valueError:
    try:
      if floatString[-3] in '+-':
        exponent = floatString[-2:]
        return float(floatString[:-4]) * 10 ** float(exponent)
    except ValueError as valueError2:
      msg = 'could not convert string to float'
      if msg in str(valueError) and msg in str(valueError2):
        return np.nan
