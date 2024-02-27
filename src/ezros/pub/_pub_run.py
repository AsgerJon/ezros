"""Runs the pub window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication
from icecream import ic
from vistutils.dirs import getProjectRoot

here = os.getcwd()
root = os.path.join(here, '..', '..', )
root = os.path.normpath(root)
sys.path.append(root)
ic(root)
from ezros.pub import PubMain

if __name__ == '__main__':
  app = QApplication(sys.argv)
  window = PubMain()
  window.show()
  sys.exit(app.exec_())
