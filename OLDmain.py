"""Main Tester Script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from pyperclip import copy

from ezros.gui.shortnames import ColorSpace
from ezros.gui.windows import MainWindow
from _dep.morevistutils import Nunc
from _dep.morevistutils import Singleton
from tester_class_01 import Tester, Tester2
from tester_class_02 import Owner

from tester_class_01 import Taboo


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
  """bla"""
  lol = [69, 420, ], dict(lmao=True)
  try:
    lol.yolo()
  except AttributeError as e:
    print(e)


def tester03() -> None:
  """bla"""
  try:
    object.__init_subclass__()
  except Exception as e:
    print(e)


def tester04() -> None:
  """bla"""

  lmao = Singleton()
  yolo = Singleton()

  print(lmao is yolo)


def tester05() -> None:
  """bla"""

  lmao = Tester('LMAO')
  yolo = Tester('YOLO')
  print(lmao is yolo)
  lmao = Tester2('LMAO')
  yolo = Tester2('YOLO')
  print(lmao is yolo)


def tester06() -> None:
  """Argument related error messages"""

  def lol(a: int, b: int) -> int:
    return a + b

  try:
    lol()
  except TypeError as e:
    print(e)


def tester07() -> None:
  """Argument related error messages"""

  print(Nunc(7))


def tester08() -> None:
  """Argument related error messages"""

  owner = Owner()
  +Taboo
  print(Taboo, owner)


def tester09() -> None:
  """Argument related error messages"""
  for lmao in 777:
    print(lmao)


def tester10() -> None:
  """qt mnousebuttons again"""
  for button in Qt.MouseButton:
    print(button, button.value, type(button.value))

  left = Qt.MouseButton.LeftButton
  right = Qt.MouseButton.RightButton
  test = Qt.MouseButtons(left | right)
  print(test, test.value, type(test.value))


def tester11() -> None:
  """Color space test"""

  allCols = []
  data = []

  for (key, val) in ColorSpace.__dict__.items():
    if not key.startswith('_'):
      if key[0].isupper():
        allCols.append("""'%s',""" % key)
        r, g, b = val.red(), val.green(), val.blue()
        # if max(r, g, b) - min(r, g, b) < 16:
        if r < 64 and g > 64 and b > 128:
          line = """'%s': QColor(%s, %s, %s, ),""" % (key, r, g, b)
          data.append((line, r, g, b))
  data.sort(key=lambda x: x[1] + x[2] + x[3])
  lines = [i[0] for i in data]
  lines.insert(0, '{')
  lines.append('}')
  print('\n'.join(lines))
  copy('\n'.join(lines))


def tester12() -> None:
  """LMAO"""
  try:
    object.__init_subclass__(stfu=True)
  except Exception as Karen:
    print(Karen.__class__.__qualname__)
    print(Karen)

  kwargs = dict(stfu=True, lmao=True, yolo=True)
  args = [69, 420, 1337, 'lmao']

  names = []

  for (key, val) in object.__dict__.items():
    keyMsg = '%18s | ' % key
    valMsg = '%28s | ' % val.__class__.__qualname__
    if val.__class__.__qualname__ in ['wrapper_descriptor',
                                      'method-wrapper',
                                      'method_descriptor',
                                      'builtin_function_or_method',
                                      'classmethod_descriptor']:
      names.append(key)
      print(keyMsg, valMsg, 'added')
    else:
      print(keyMsg, valMsg, 'not added')


def tester13() -> None:
  """lmao"""
  lol = [
    object.__new__,
    object.__repr__,
    object.__hash__,
    object.__str__,
    object.__getattribute__,
    object.__setattr__,
    object.__delattr__,
    object.__lt__,
    object.__le__,
    object.__eq__,
    object.__ne__,
    object.__gt__,
    object.__ge__,
    object.__init__,
    object.__reduce_ex__,
    object.__reduce__,
    object.__getstate__,
    object.__subclasshook__,
    object.__init_subclass__,
    object.__format__,
    object.__sizeof__,
    object.__dir__,
  ]
  args = [69, 420, 1337, 'lmao']
  kwargs = dict(stfu=True, lmao=True, yolo=True)

  names = []
  funcs = []

  for (key, val) in object.__dict__.items():
    if val.__class__.__qualname__ in ['wrapper_descriptor',
                                      'method-wrapper',
                                      'method_descriptor',
                                      'builtin_function_or_method',
                                      'classmethod_descriptor']:
      names.append(key)
      funcs.append(getattr(val, '__func__', val))

  for name, func in zip(names, funcs):
    try:
      func(*args, **kwargs)
      print('Success:', name)
    except Exception as e:
      print('Error:', name, e)

  print(object.__getattr__)


if __name__ == '__main__':
  tester13()
