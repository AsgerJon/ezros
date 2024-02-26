"""Tester"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QPaintEvent, QPainter
from PySide6.QtWidgets import QWidget


class AbstractDescriptor:
  """AbstractDescriptor provides a base class for descriptor classes."""

  __field_name__ = None
  __field_owner__ = None

  def __set_name__(self, owner: type, name: str) -> None:
    """Set the name of the field."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def getPrivateName(self) -> str:
    """Return the private name of the field."""
    return '_%s' % self.__field_name__

  def __get__(self, instance: Any, owner: type, **kwargs) -> int:
    """Getter function for the field"""
    if instance is None:
      return self
    pvtName = self.getPrivateName()
    return getattr(instance, pvtName, 0)

  def __set__(self, instance: Any, value: int) -> None:
    """Setter function for the field"""
    pvtName = self.getPrivateName()
    setattr(instance, pvtName, value)

  def __delete__(self, instance: Any) -> None:
    """Deleter function for the field"""
    pvtName = self.getPrivateName()
    delattr(instance, pvtName)


class IntField(AbstractDescriptor):
  """IntField provides a number valued descriptor class of type int."""


class Color(AbstractDescriptor):
  """Color provides a QColor valued descriptor class."""
  red = IntField()
  green = IntField()
  blue = IntField()


class Pen:
  """Pen class for the PenField class."""
  color = Color()
  lineWidth = IntField()


class Brush:
  """Brush class for the BrushField class."""
  color = Color()


class Widget(QWidget):
  """Widget class for the BrushField class."""
  borderPen = Pen()
  fillBrush = Brush()

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint event for the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setPen(self.borderPen.color)
    painter.setBrush(self.fillBrush.color)
    viewRect = painter.viewport()
    painter.drawRect(viewRect)
    painter.end()
