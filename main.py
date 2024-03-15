"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from icecream import ic

from ezros.app import MainWindow
from tester_class_02 import TestWindow


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, ]
  for item in stuff:
    print(item)


fb = lambda n: '' if n % 5 else 'Fizz' + '' if n % 3 else 'Buzz' or str(n)


def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()


def tester02() -> None:
  """lmao"""


if __name__ == '__main__':
  tester01()
