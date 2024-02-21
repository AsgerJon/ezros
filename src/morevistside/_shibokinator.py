"""The sbibokinator function identifies objects that are subclasses of
QObject or instances of such classes. Returning True if so."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtWidgets import QWidget

ObjectType = type(QWidget)


def _shibokinator(obj: Any) -> Any:
  """The sbibokinator function identifies objects that are subclasses of
  QObject or instances of such classes. Returning True if so."""
  if isinstance(obj, type):
    if type(obj) is ObjectType:
      return True, type(obj).__qualname__
    if type(obj) is type:
      return False, type(obj).__qualname__
  return shibokinator(type(obj))


def shibokinator(obj: Any) -> tuple:
  """The sbibokinator function identifies objects that are subclasses of
  QObject or instances of such classes. Returning True if so."""
  return _shibokinator(obj)
