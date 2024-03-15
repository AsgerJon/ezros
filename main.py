"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from icecream import ic

from ezros.app import MainWindow

#
# import os
# import sys
#
# import logging
# from PySide6.QtCore import Qt
# from PySide6.QtWidgets import QApplication
#

# from ezros.app import MainWindow
ic.configureOutput(includeContext=True)

#
fb = lambda n: (('' if n % 5 else 'Fizz') + ('' if n % 3 else 'Buzz') or
                str(n))


def tester00() -> None:
  stuff = [os, sys, 'hello world', Qt, ]
  for item in stuff:
    print(item)

  for (i, _) in enumerate(range(100)):
    print('%03d' % i, fb(_))


#
#
def tester01() -> None:
  """Main application tester"""

  app = QApplication(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()


if __name__ == '__main__':
  tester01()
