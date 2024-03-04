"""LMAO"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


def tester05() -> None:
  """lmao"""

  import sys
  import os
  here = os.getcwd()
  sys.path.append(os.path.join(here, 'src'))
  print(sys.path)
  from _dep.morevistutils import AbstractDescriptor

  class A:
    b = AbstractDescriptor()

  a = A()
  print(a.b)
  print(AbstractDescriptor.__dict__)


if __name__ == '__main__':
  tester05()
