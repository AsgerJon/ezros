"""The parseText function text from arguments."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.text import stringList


def parseText(*args, **kwargs) -> str:
  """Parse the text from arguments."""
  textKeys = stringList("""text, msg, message, label, title""")
  for key in textKeys:
    if key in kwargs:
      val = kwargs[key]
      if isinstance(val, str):
        return val
  else:
    for arg in args:
      if isinstance(arg, str):
        return arg
