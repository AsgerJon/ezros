"""The 'addMsgType' function receives a namespace defining a Python class
or the class itself, creates a ROS message file from it, and adds it to the
specified ROS package. If a namespace is given, the name of the type must
be given in the third positional argument, at the keyword argument
'clsName' or as value at '__name__' in the given namespace. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from vistutils.waitaminute import typeMsg

from ezros.env import changeDir
from ezros.rosutils import generateRosPackage
from ezros.rosutils._generate_ros_package import getRosPackageDir


def addMsgType(*args) -> None:
  """The 'addMsgType' function receives a namespace defining a Python class
  or the class itself, creates a ROS message file from it, and adds it to the
  specified ROS package. If a namespace is given, the name of the type must
  be given in the third positional argument, at the keyword argument
  'clsName' or as value at '__name__' in the given namespace. """
  namespace, rosPackage, clsName = None, None, None
  for arg in args:
    if isinstance(arg, dict) and namespace is None:
      namespace = arg
    elif isinstance(arg, str) and rosPackage is None:
      rosPackage = arg
    elif isinstance(arg, str) and clsName is None:
      clsName = arg
  if rosPackage is None:
    e = """Missing required argument: 'rosPackage'!"""
    raise ValueError(e)
  if namespace is None:
    e = """Missing required argument: 'namespace'!"""
    raise ValueError(e)
  clsName = namespace.get('__name__', clsName)
  if clsName is None:
    e = """Missing required argument: 'clsName'!"""
    raise ValueError(e)
  annotations_ = namespace.get('__annotations__', {})
  namespace |= annotations_
  lines = []
  for (key, val) in namespace.items():
    if key.startswith('__') and key.endswith('__'):
      continue
    if isinstance(val, str):
      lines.append('%s %s' % (val, key))
    elif isinstance(val, type):
      lines.append('%s %s' % (val.__name__, key))
    else:
      e = typeMsg('val', val, type)
      raise TypeError(e)
  generateRosPackage(rosPackage, )
  rosPackageDir = getRosPackageDir(rosPackage, )
  msgFile = changeDir(rosPackageDir, '%s.msg' % clsName)
  if not os.path.exists(msgFile):
    with open(msgFile, 'w') as f:
      f.write('\n'.join(lines))
      f.write('\n')
