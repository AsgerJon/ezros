"""Auto-generated stub file for the main window.
Sat Mar 2 02:33:03 2024"""
# MIT Licence
# Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from types import FunctionType as function

from PySide6.QtCore import QMetaObject
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow

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
  paintTimer: Wait
  createActionStub: function
  startPump: function
  connectActions: function
  updateState: function
  dataCallback: function
  pumpControl: function
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
  data: Wait
  toggle: Wait
  initUI: function
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
