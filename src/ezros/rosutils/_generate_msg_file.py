"""The generateMsgFile function generates a new message file in the specified
package, with the specified name and message definition."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from vistutils.waitaminute import typeMsg

from ezros.env import changeDir
from ezros.rosutils import CATKIN_WS, resolveMsgType, getRosPackageDir


def generateMsgFile(package: str, cls: type) -> None:
  """Generate a new message file in the specified package, with the specified
  name and message definition."""
  packageDir = changeDir(CATKIN_WS, package)
  packageDir = getRosPackageDir(package)
  if not os.path.exists(packageDir):
    os.makedirs(packageDir)
  if os.path.isfile(packageDir):
    e = """Expected directory at: '%s', but found a file!"""
    raise NotADirectoryError(e % packageDir)
  msgFile = changeDir(packageDir, 'msg', '%s.msg' % cls.__name__)
  if hasattr(cls, '__slots__') and hasattr(cls, '_slot_types'):
    base = dict(zip(cls.__slots__, cls._slot_types))
  elif getattr(cls, '__annotations__', None) is not None:
    base = cls.__annotations__
    if not isinstance(base, dict):
      e = typeMsg('base', base, dict)
      raise TypeError(e)
  else:
    base = {}
  for (key, val) in cls.__dict__.items():
    if key.startswith('__') and key.endswith('__'):
      continue
    if callable(val):
      continue
    if isinstance(val, type):
      base[key] = val
    elif isinstance(val, str):
      base[key] = resolveMsgType(val) or str
    else:
      base[key] = type(val)

  lines = []
  for (key, val) in base.items():
    lines.append('%s %s' % (val.__name__, key))

  with open(msgFile, 'w') as f:
    f.write('\n'.join(lines))
    f.write('\n')
