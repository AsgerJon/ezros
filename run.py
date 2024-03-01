"""Wraps the actual main call"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
import os

from PySide6.QtCore import QObject
from PySide6.QtWidgets import QWidget, QApplication
from rospy import init_node

from ezros.sub import SubMain

msgsPath = '~/home/asger/catkin_ws/devel/lib/python3.11/site-packages/'
msgsPath = os.path.expanduser(msgsPath)
sys.path.insert(0, msgsPath)

if __name__ == '__main__':
  try:
    from main import tester01

    init_node('Test', anonymous=False, )
    app = QApplication(sys.argv)
    main = SubMain()
    main.show()
    sys.exit(app.exec())

  except ModuleNotFoundError as moduleNotFoundError:
    e = """The main failed because of a missing module with the following 
    error message:\n%s""" % moduleNotFoundError
    e2 = """The main search through the following directories for 
    packages: (sys.path):\n%s""" % ('\n'.join(sys.path))
    print('%s\n%s' % (e, e2))
