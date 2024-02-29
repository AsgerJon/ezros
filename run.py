"""Wraps the actual main call"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
import os

msgsPath = '~/home/asger/catkin_ws/devel/lib/python3.11/site-packages/'
msgsPath = os.path.expanduser(msgsPath)
sys.path.insert(0, msgsPath)

if __name__ == '__main__':
  try:
    from main import tester01

    tester01()
  except ModuleNotFoundError as moduleNotFoundError:
    e = """The main failed because of a missing module with the following 
    error message:\n%s""" % moduleNotFoundError
    e2 = """The main search through the following directories for 
    packages: (sys.path):\n%s""" % ('\n'.join(sys.path))
    print('%s\n%s' % (e, e2))
