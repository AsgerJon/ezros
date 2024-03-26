"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from icecream import ic
import msgs.msg as msg
import numpy as np

from ezros.app import App, MainWindow

ic.configureOutput(includeContext=True, )


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, msg]
  for item in stuff:
    print(item)


fb = lambda n: '' if n % 5 else 'Fizz' + '' if n % 3 else 'Buzz' or str(n)


def tester01() -> None:
  """Main application tester"""

  app = App(sys.argv)
  if app.loadEnv():
    main = MainWindow()
    main.show()
    app.exec()


def tester02() -> None:
  """lmao"""
  for item in dir(msg):
    print(item)


def tester03() -> None:
  """lmao"""
  test = msg.AuxCommand()
  print(test)


if __name__ == '__main__':
  tester03()
