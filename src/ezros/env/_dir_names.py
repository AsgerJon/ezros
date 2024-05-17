"""This file provides named directories. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os


def getParentDir(fileDir: str, levels: int = None) -> str:
  """Returns the parent directory of the given file or dir. If file is
  given, the dir containing the file is returned. If dir is given, the
  parent dir is returned. """
  levels = 1 if levels is None else levels
  if os.path.isfile(fileDir):
    return getParentDir(os.path.dirname(fileDir), levels - 1)
  if not levels:
    return fileDir
  if fileDir == '/':
    return '/'
  return getParentDir(os.path.dirname(fileDir), levels - 1)


def changeDir(fileDir: str, *child: str) -> str:
  """Changes directory to the child directory of the given directory. """
  if os.path.isfile(fileDir):
    if child:
      e = """The given directory is a file, not a directory. """
      raise NotADirectoryError(e)
    return fileDir
  children = [*child]
  if not children:
    return fileDir
  out = os.path.join(fileDir, children.pop(0))
  return changeDir(out, *children)


_here = os.path.abspath(os.path.dirname(__file__))

SITE_PACKAGES = getParentDir(_here, 4)
EZ_ROOT = getParentDir(_here, 3)
