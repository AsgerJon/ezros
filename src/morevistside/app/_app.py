"""The 'app' module provides subclasses for the application objects in the
Qt framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

App = type('App', (), {'lmao': True})
#
# from typing import Callable, Any
#
# from PySide6.QtCore import QObject
# from PySide6.QtWidgets import QApplication, QMainWindow
# from icecream import ic
# from vistutils.waitaminute import typeMsg
#
# ic.configureOutput(includeContext=True)
#
#
# class App(QApplication):
#   """The App class subclasses QApplication and provides a default
#   implementation for the main application object."""
#
#   def __init__(self, *args, **kwargs) -> None:
#     self.__main_window__ = None
#     self.__event_filter__ = None
#     mainWindow, eventFilter = None, None
#     for arg in args:
#       ic(arg)
#       if isinstance(arg, type):
#         if issubclass(arg, QMainWindow):
#           if mainWindow is None:
#             mainWindow = arg
#       elif isinstance(arg, QObject) and eventFilter is None:
#         eventFilter = arg
#       if mainWindow is not None and eventFilter is not None:
#         break
#     else:
#       if mainWindow is None:
#         e = """Missing required main window argument"""
#         raise ValueError(e)
#     if issubclass(mainWindow, QMainWindow):
#       self.__main_window__ = mainWindow
#     if eventFilter is not None:
#       if isinstance(eventFilter, QObject):
#         self.__event_filter__ = eventFilter
#       else:
#         e = typeMsg('__event_filter__', self.__event_filter__, QObject)
#         raise TypeError(e)
#     strArgs = [arg for arg in args if isinstance(arg, str)]
#     QApplication.__init__(self, *strArgs)
#
#   def getMainWindow(self) -> QMainWindow:
#     """Getter-function for the main window."""
#     return self.__main_window__()
#
#   def getEventFilter(self) -> QObject:
#     """Getter-function for the event filter."""
#     return self.__event_filter__
#
#   def exec(self) -> int:
#     """Executes the application."""
#     if not isinstance(self, QApplication):
#       e = typeMsg('self', self, QApplication)
#       raise TypeError(e)
#     if hasattr(QApplication.exec, '__self__'):
#       return QApplication.exec()
#     return QApplication.exec(self)
#
#   def run(self) -> int:
#     """Runs the application"""
#     self.getMainWindow().show()
#     return self.exec()
#
#   def __rshift__(self, calLMeMaybe: Callable) -> Any:
#     """Runs the application and passes the return code to the given
#     callable."""
#     return calLMeMaybe(self.run())
