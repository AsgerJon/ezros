"""Auto-generated stub file for the main window.
Tue Feb 27 16:54:12 2024"""
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from types import FunctionType as function

from PySide6.QtCore import QMetaObject
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow

from morevistutils.fields import Later


method_descriptor = type(QMainWindow.addDockWidget)
classmethod_descriptor = type(classmethod)
class MainWindow(QMainWindow):
  """The main application window class."""
  
# Actions contribution from MainWindow
  new: QAction
  open: QAction
  save: QAction
  saveAs: QAction
  eXIT: QAction
  undo: QAction
  redo: QAction
  cut: QAction
  copy: QAction
  paste: QAction
  delete: QAction
  selectAll: QAction
  aboutQtAction: QAction
  aboutPythonAction: QAction
  debug01Action: QAction
  debug02Action: QAction
  debug03Action: QAction
  debug04Action: QAction
  debug05Action: QAction
  debug06Action: QAction
  debug07Action: QAction
  debug08Action: QAction
  debug09Action: QAction
  
# Namespace contribution from MainWindow
  subscribe: Signal
  paintTimer: Wait
  createActionStub: function
  connectActions: function
  updateState: function
  testPaint: function
  testPlot: function
  debug01Func: function
  debug02Func: function
  debug03Func: function
  debug04Func: function
  debug05Func: function
  debug06Func: function
  staticMetaObject: QMetaObject
  
# Actions contribution from LayoutWindow
  new: QAction
  open: QAction
  save: QAction
  saveAs: QAction
  eXIT: QAction
  undo: QAction
  redo: QAction
  cut: QAction
  copy: QAction
  paste: QAction
  delete: QAction
  selectAll: QAction
  aboutQtAction: QAction
  aboutPythonAction: QAction
  debug01Action: QAction
  debug02Action: QAction
  debug03Action: QAction
  debug04Action: QAction
  debug05Action: QAction
  debug06Action: QAction
  debug07Action: QAction
  debug08Action: QAction
  debug09Action: QAction
  
# Namespace contribution from LayoutWindow
  baseWidget: Wait
  baseLayout: Wait
  welcomeLabel: Wait
  goodbyeLabel: Wait
  state: Wait
  plot: Wait
  data: Wait
  initUI: function
  connectActions: function
  staticMetaObject: QMetaObject
  
# Actions contribution from BaseWindow
  new: QAction
  open: QAction
  save: QAction
  saveAs: QAction
  eXIT: QAction
  undo: QAction
  redo: QAction
  cut: QAction
  copy: QAction
  paste: QAction
  delete: QAction
  selectAll: QAction
  aboutQtAction: QAction
  aboutPythonAction: QAction
  debug01Action: QAction
  debug02Action: QAction
  debug03Action: QAction
  debug04Action: QAction
  debug05Action: QAction
  debug06Action: QAction
  debug07Action: QAction
  debug08Action: QAction
  debug09Action: QAction
  
# Namespace contribution from BaseWindow
  mainMenuBar: Wait
  addAction: function
  getOwnedActions: function
  initUI: function
  connectActions: function
  createActionStub: function
  show: function
  aboutPython: function
  staticMetaObject: QMetaObject