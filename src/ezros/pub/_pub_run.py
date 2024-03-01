"""Runs the pub window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication
from icecream import ic
from rospy import init_node

here = os.getcwd()
root = os.path.join(here, '..', '..', )
root = os.path.normpath(root)
sys.path.append(root)

if __name__ == '__main__':
  nodeName = 'test'
  init_node(nodeName, anonymous=True)

  app = QApplication(sys.argv)
  from ezros.pub import PubMain

  window = PubMain()
  window.show()
  sys.exit(app.exec())
