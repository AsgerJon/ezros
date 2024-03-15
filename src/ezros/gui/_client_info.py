"""ClientInfo provides static information about the client."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any, Never

from PySide6.QtGui import QColor, QPaintEvent, QPainter
from vistside.core import BrushField, parsePen, Black, Tight
from vistside.widgets import BaseWidget, BaseLayoutField, LabelField, \
  LabelWidget
from vistutils.fields import unParseArgs, Wait


class Field:
  """LMAO"""

  __positional_args__ = None
  __keyword_args__ = None
  __field_name__ = None
  __field_type__ = None
  __field_owner__ = None
  __lazy_setters__ = None

  @classmethod
  def __class_getitem__(cls, innerCls: type) -> Any:
    """Returns a new Field with the inner class as the field type."""
    return cls(innerCls)

  def __init__(self, cls: type) -> None:
    """Initializes the field."""
    self.__field_type__ = cls

  def __call__(self, *args, **kwargs) -> Any:
    """Returns the field."""
    self.__positional_args__ = [*args, ]
    self.__keyword_args__ = {**kwargs, }
    return self

  def __set_name__(self, owner: type, name: str) -> None:
    """Sets the name of the field."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getFieldName(self, ) -> str:
    """Getter-function for private field name"""
    return self.__field_name__

  def _getPrivateFieldName(self, ) -> str:
    """Getter-function for private field name"""
    return '_%s' % self.__field_name__

  def _getArgs(self) -> list:
    """Getter-function for positional arguments"""
    return self.__positional_args__

  def _getKwargs(self) -> dict:
    """Getter-function for keyword arguments"""
    return self.__keyword_args__

  def _instantiate(self, instance: object, ) -> Any:
    """Instantiates the field."""
    pvtName = self._getPrivateFieldName()
    value = self.__field_type__(*self._getArgs(), **self._getKwargs())
    setattr(instance, pvtName, value)

  def __get__(self, instance: object, owner: type, **kwargs) -> Any:
    """Returns the field."""
    if instance is None:
      return self
    pvtName = self._getPrivateFieldName()
    if getattr(instance, pvtName, None) is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._instantiate(instance)
      return self.__get__(instance, owner, _recursion=True, )
    return getattr(instance, pvtName)

  def __set__(self, *_) -> Never:
    """Must be implemented in subclass"""
    e = """Instances of Field cannot be set. """
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Must be implemented in subclass"""
    e = """Instances of Field cannot be deleted. """
    raise TypeError(e)


class ClientInfo(BaseWidget):
  """The ClientInfo class provides static information about the client."""

  __fallback_uri__ = 'http://localhost:11311'
  __ros_master_uri__ = os.environ.get('ROS_MASTER_URI', __fallback_uri__)

  fillBrush = BrushField(QColor(255, 255, 255, 63))

  baseLayout = BaseLayoutField(layout='vertical')

  rosUriField = LabelField('ROS Master URI: ')
  clientLocation = Field[LabelWidget]('Location: ')

  def __init__(self, *args, **kwargs) -> None:
    """Create a new ClientInfo."""
    BaseWidget.__init__(self, *args, **kwargs)
    uri = self.__ros_master_uri__.replace('http://', '')
    uri = uri.replace(':11311', '')
    self.rosUriField.innerText = uri
    clientName = kwargs.get('client', None)
    location = kwargs.get('location', None)
    self.clientLocation.innerText = 'Location'
    self.initUi()

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.rosUriField)
    self.baseLayout.addWidget(self.clientLocation)
    self.setLayout(self.baseLayout)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget."""
    painter = QPainter()
    painter.begin(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    viewRect = painter.viewport()
    # # # # # # # # # # # # # # # # #
    #  fill background
    painter.setPen(self.emptyPen)
    painter.setBrush(self.fillBrush)
    painter.drawRect(viewRect)
    # # # # # # # # # # # # # # # # #
    #  draw border
    painter.setPen(parsePen(Black, 1))
    painter.setBrush(self.emptyBrush)
    painter.drawRect(viewRect)
    painter.end()

  @classmethod
  def getDefault(cls, *args, **kwargs) -> ClientInfo:
    """Returns the default value for the field."""
    return cls(*args, **kwargs)

  def apply(self, value: Any) -> ClientInfo:
    """Applies the value to the field."""
    args, kwargs = unParseArgs(value)
    return self


class ClientInfoField(Wait):
  """ClassInfoField provides static information about the class."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the descriptor"""
    Wait.__init__(self, ClientInfo, *args, **kwargs)
