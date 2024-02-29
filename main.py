"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtCore import QEvent, Qt
from PySide6.QtWidgets import QApplication
from icecream import ic

from ezros.gui.windows import MainWindow


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt]
  for item in stuff:
    print(item)


def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()


def tester02() -> None:
  """Events"""
  eventNames = [e.name for e in QEvent.Type]
  eventValues = [e.value for e in QEvent.Type]
  n = max([len(name) for name in eventNames])
  fmtSpec = '%%03d | %%%ds | %%d' % (n,)
  for (i, e) in enumerate(QEvent.Type):
    if 'enter' in e.name.lower():
      print(fmtSpec % (i, e.name, e.value))


def tester03() -> None:
  """lmao"""


if __name__ == '__main__':
  tester01()
