"""Main Tester Script"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
from typing import Callable

import numpy as np

from ezros.app import App, MainWindow
from PySide6.QtCore import Qt
from icecream import ic
from msgs.msg import Float32Stamped
import msgs.msg as msg
import std_msgs.msg as std_msg
from vistutils.text import monoSpace

from ezros.utils import Announcer

ic.configureOutput(includeContext=True, )


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, ]
  for item in stuff:
    print(item)


def tester01() -> int:
  """Main application tester"""
  app = App(sys.argv)
  mainWindow = MainWindow()
  mainWindow.show()
  return app.exec()


def tester02() -> None:
  """lmao"""


def tester03() -> None:
  """lmao"""


def tester06() -> None:
  """THESE arrays fuck me"""
  """127.0.0.1	localhost
127.0.1.1	TMR
192.168.1.85	tinybox9542
192.168.1.209	tinybox6112
# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
"""


def main(callMeMaybe: Callable) -> Announcer:
  """This function decides the test function called"""
  mainAnnouncer = Announcer('Running: %s' % callMeMaybe.__name__)
  res = callMeMaybe()
  mainAnnouncer.exit('Completed with res: %d' % res)
  return mainAnnouncer


if __name__ == '__main__':
  tester01()
