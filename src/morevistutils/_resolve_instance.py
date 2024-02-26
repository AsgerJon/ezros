"""The resolveInstance function attempts to instantiate a class from the
class itself. For common types, a look up table provides appropriate
default instances. For more complex type, a call to the inferred
constructor is instead attempted. Finally, custom classes may provide a
callable at the name '__create_default__'. If an attribute is present
here, this alone will be attempted."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

explicitName = '__create_default__'


def resolveInstance(cls: type) -> object:
  """The resolveInstance function attempts to instantiate a class from the
  class itself. For common types, a look up table provides appropriate
  default instances. For more complex type, a call to the inferred
  constructor is instead attempted."""
  if hasattr(cls, explicitName):
    return getattr(cls, explicitName)()
  if cls is int:
    return 0
  if cls is float:
    return 0.0
  if cls is complex:
    return 0 + 0j
  if cls is str:
    return ""
  if cls is bool:
    return False
  if cls is list:
    return []
  if cls is dict:
    return {}
  if cls is set:
    return set()
  if cls is tuple:
    return ()
  if cls is type(None):
    return None
  try:
    return cls()
  except Exception as exception:
    e = """Unable to resolve instance for type '%s'!""" % cls.__qualname__
    raise TypeError(e) from exception
