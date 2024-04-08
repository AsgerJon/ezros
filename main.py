"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

sys.path.append(os.path.join(sys.path[0], 'src'))

import numpy as np

from ezros.app import App, MainWindow
from PySide6.QtCore import Qt
from icecream import ic
from msgs.msg import Float32Stamped
import msgs.msg as msg
import std_msgs.msg as std_msg
from vistutils.text import monoSpace

from ezros.rosutils import Commander

ic.configureOutput(includeContext=True, )


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', Qt, msg]
  for item in stuff:
    print(item)


fb = lambda n: '' if n % 5 else 'Fizz' + '' if n % 3 else 'Buzz' or str(n)


def tester01() -> None:
  """Main application tester"""
  ic(sys.path)
  app = App(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()


def tester02() -> None:
  """lmao"""
  for item in dir(msg):
    print(item)


def tester03() -> None:
  """lmao"""
  test = msg.Float32Stamped()
  print(test.__slots__)


def tester04() -> None:
  """commander"""
  userInput = input('Please provide topic name:\n')
  commander = Commander(userInput, 'TestNode', )
  commander.createPublisher()
  userInput = '0'
  while userInput.isnumeric():
    userInput = input('Enter a duration length in milliseconds: \n')
    try:
      output = """Activating the commander with a duration of '%.3f' 
      milliseconds""" % float(userInput)
      print(monoSpace(output))
      commander.activate(float(userInput))
    except ValueError as valueError:
      print(str(valueError))
      print('Ending test now')
      sys.exit(0)
  print('lol, not a number')


def tester05() -> None:
  """LMAO"""
  test = std_msg.Header()
  print(test.__slots__)
  print(test._get_types())
  print(test)
  yolo = Float32Stamped(std_msg.Header(), 69)
  print(55 * '*')
  print(yolo)


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


if __name__ == '__main__':
  tester01()
