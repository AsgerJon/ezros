"""The flags in this module exposes the state of the mouse buttons at the
moment their getter is called."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt, QObject, QEvent, Signal
from PySide6.QtGui import QGuiApplication
from icecream import ic
from vistutils.fields import AbstractField
from vistutils.text import stringList


class _ButtonField(AbstractField):
  """This descriptor class provides getters for the current state of the
  mouse buttons."""

  @staticmethod
  def _perfectLog2(n: int) -> bool:
    """This function tests if a number is a perfect power of 2."""
    return n & (n - 1) == 0

  def __init__(self, *args) -> None:
    AbstractField.__init__(self, *args)
    buttonQt = None
    buttonStr = None
    buttonInt = None
    for arg in args:
      if isinstance(arg, str):
        if arg.lower() in stringList("""left, middle, right, back, 
          forward"""):
          buttonStr = arg
      elif isinstance(arg, int):
        if not arg & (arg - 1):
          buttonInt = arg
      elif isinstance(arg, Qt.MouseButton):
        buttonQt = arg
      if any([b is not None for b in [buttonQt, buttonStr, buttonInt]]):
        break
    else:
      e = """Unable to parse button!"""
      raise ValueError(e)


class Mouse(QObject):
  """The MouseButtons class exposes the state of the mouse buttons at the
  moment their getter is called."""

  __mouse_button_events__ = [QEvent.MouseButtonPress,
                             QEvent.MouseButtonRelease,
                             QEvent.MouseButtonDblClick]

  buttonEvent = Signal()
  buttonChange = Signal(Qt.MouseButton, Qt.MouseButton)

  def event(self, event_: QEvent) -> bool:
    """Reimplementation to handle mouse events."""
    if event_.type() in self.__mouse_button_events__:
      ic(QGuiApplication.mouseButtons())
      self.buttonChange.emit()
    return False
 