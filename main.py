"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

import logging
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from ezros.app import MainWindow
from PySide6.QtGui import QFontDatabase


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, ]
  for item in stuff:
    print(item)


fb = lambda n: '' if n % 5 else 'Fizz' + '' if n % 3 else 'Buzz' or str(n)


def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)

  available_families = QFontDatabase().families()

  for family in available_families:
    print(family)

  main = MainWindow()
  main.show()
  app.exec()


def tester02() -> None:
  """lmao"""


if __name__ == '__main__':
  tester01()
