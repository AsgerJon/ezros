"""The resolveType function receives a string valued name of a type and
attempts to resolve the type. First, it searches for the type in the
globals() dictionary. Next it searches for the type in the builtins.
Finally, it searches for the name in sys.path (under development)."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


def resolveType(name: str) -> type:
  """The resolveType function receives a string valued name of a type and
  attempts to resolve the type. First, it searches for the type in the
  globals() dictionary. Next it searches for the type in the builtins.
  Finally, it searches for the name in sys.path (under development)."""
  if name in globals():
    return globals()[name]
  if name in __builtins__:
    return __builtins__[name]
  e = """Name: '%s' is not assigned to a type in globals() or 
  __builtins__!"""
  e2 = """Type resolution against sys.path is not yet implemented!"""
  raise NameError(e) from NotImplementedError(e2)
