"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtCore import Qt, QProcess
from PySide6.QtWidgets import QApplication
import msgs as msg

from morevistside.windows import BaseWindow


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, msg]
  for item in stuff:
    print(item)


fb = lambda n: '' if n % 5 else 'Fizz' + '' if n % 3 else 'Buzz' or str(n)


#
def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)
  main = BaseWindow()
  main.show()
  app.exec()


#
#
# def tester02() -> None:
#   """Events"""
#   eventNames = [e.name for e in QEvent.Type]
#   eventValues = [e.value for e in QEvent.Type]
#   n = max([len(name) for name in eventNames])
#   fmtSpec = '%%03d | %%%ds | %%d' % (n,)
#   for (i, e) in enumerate(QEvent.Type):
#     if 'enter' in e.name.lower():
#       print(fmtSpec % (i, e.name, e.value))
#
#
# def tester03() -> None:
#   """lmao"""
#
#   import lmao.yolo as lmao
#   yoloCode = []
#   for (key, val) in lmao.__dict__.items():
#     if isinstance(val, type) and key[0].isupper():
#       entry = """%s = type('%s', (genpy.Message,), {})"""
#       yoloCode.append(entry % (key, key,))
#   yoloCode = '\n'.join(yoloCode)
#   copy(yoloCode)


def tester04() -> None:
  """lmao"""
  sub = QProcess()

  sub.finished.connect(lambda: print('finished'))
  sub.start('python3', ['_sub_run.py'])


if __name__ == '__main__':
  tester01()
