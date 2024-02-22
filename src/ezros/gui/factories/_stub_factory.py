"""The stubFactory writes out a stub file for the class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


def header() -> str:
  """The header of the stub file."""
  h = """\"\"\"Auto-generated stub file for the main window.\"\"\"<br>
  #  MIT Licence<br>
  #  Copyright (c) 2024 Asger Jon Vistisen<br>
  from __future__ import annotations<br>
  <br>
  from enum import EnumType<br>
  from types import FunctionType as function<br>
  <br>
  from PySide6.QtCore import QMetaObject, Signal<br>
  from PySide6.QtGui import QAction<br>
  from PySide6.QtWidgets import QMainWindow<br>
  <br>
  from OLDmorevistutils.fields import Later<br>
  <br><br>
  method_descriptor = type(QMainWindow.addDockWidget)<br>
  classmethod_descriptor = type(classmethod)<br>
  class MainWindow(QMainWindow):<br>
    <tab>\"\"\"The main application window class.\"\"\"<br>
    
  """
  return '<br>'.join([line.strip() for line in h.split('<br>')])