"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

import numpy as np
from PySide6.QtWidgets import QApplication
from numpy import full, nan, complex64

from ezros.gui.windows import MainWindow
from morevistutils import DataEcho


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world']
  for item in stuff:
    print(item)


def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()


def tester02() -> None:
  """Numpyrithmetic"""
  nanny = full((128,), nan, dtype=complex64)
  print(nanny.shape)
  J = np.ones_like(nanny, dtype=np.uint8)

  print(np.sum(J - (nanny == nanny).astype(int)))


def tester03() -> None:
  """Testing dadta echo"""
  lmao = DataEcho(128)
  print(lmao)


if __name__ == '__main__':
  tester01()
