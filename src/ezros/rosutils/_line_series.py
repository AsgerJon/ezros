"""LineSeries wraps the QLineSeries."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCharts import QLineSeries
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg


class Wrapper:
  """LineSeries wraps the QLineSeries."""

  __line_series__ = None
  __field_name__ = None
  __field_owner__ = None

  @classmethod
  def __class_getitem__(cls, innerClass: type) -> type:
    """Get the inner class."""
    # return cls(innerClass)

  def __init__(self, innerClass: type) -> None:
    """Create a new instance of the inner class."""
    self.__inner_class__ = innerClass

  def getFieldOwner(self) -> type:
    """Get the owner of the field."""
    if self.__field_owner__ is None:
      e = """Field instance '%s' accessed before __set_name__ was called.
      This should not happen in most intended use-cases, unless you are 
      implementing metaclass in which case, what are you doing with your 
      life? Get some help. Somewhere else."""
      raise RuntimeError(monoSpace(e % self.__field_name__))
    if isinstance(self.__field_owner__, type):
      return self.__field_owner__
    e = typeMsg('fieldOwner', self.__field_owner__, type)
    raise TypeError(e)

  def getFieldName(self) -> str:
    """Get the name of the field."""
    if self.__field_name__ is None:
      e = """Field instance '%s' accessed before __set_name__ was called.
      This should not happen in most intended use-cases, unless you are 
      implementing metaclass in which case, what are you doing with your 
      life? Get some help. Somewhere else."""
      raise RuntimeError(monoSpace(e % self.__field_name__))
    if isinstance(self.__field_name__, str):
      return self.__field_name__
    e = typeMsg('fieldName', self.__field_name__, str)
    raise TypeError(e)

  def __set_name__(self, owner: type, name: str) -> None:
    """Set the name of the field and the owner of the field."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _createLineSeries(self, *args, **kwargs) -> None:
    """Create a line series."""
    if self.__line_series__ is None:
      self.__line_series__ = QLineSeries(*args, **kwargs)
      self.__line_series__.destroyed.connect(self._createLineSeries)
    else:
      e = """Creator function for line series instance called, but the 
      proper instance already exists. """
      raise RuntimeError(e)

  def _getLineSeries(self, **kwargs) -> QLineSeries:
    """Get the line series."""
    if self.__line_series__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createLineSeries()
      return self._getLineSeries(_recursion=True)
    if isinstance(self.__line_series__, QLineSeries):
      return self.__line_series__
    name = 'self.__line_series__'
    value = self.__line_series__
    expected = QLineSeries
    e = typeMsg(name, value, expected)
    raise TypeError(e)

  def __get__(self, instance: object, owner: type) -> QLineSeries:
    """Get the line series."""
    fieldName = self.getFieldName()
    instanceName = str(instance)
    ownerName = owner.__qualname__
    series = None
    try:
      series = self._getLineSeries()
    except RecursionError as recursionError:
      e = """When field '%s' was accessed on instance '%s' belonging to 
      class '%s', the field instance attempted to instantiate the field 
      type, but fell into a recursion!"""
      msg = monoSpace(e % (fieldName, instance, owner))
      raise RuntimeError(msg) from recursionError
    except TypeError as typeError:
      e = """When field '%s' was accessed on instance '%s' belonging to 
      class '%s', the returned value was not of the expected type!"""
      msg = monoSpace(e % (fieldName, instance, owner, typeError))
      raise RuntimeError(msg) from typeError
    except RuntimeError as yeetError:
      if 'Internal C++ object' in str(yeetError):
        self.__set_name__(owner, fieldName)
        self._createLineSeries()
        return self._getLineSeries(_recursion=True)
    except Exception as exception:
      e = """When field '%s' was accessed on instance '%s' belonging to 
      class '%s', the following uncaught exception occurred!"""
      msg = monoSpace(e % (fieldName, instance, owner))
      raise RuntimeError(msg) from exception
    if isinstance(series, QLineSeries):
      return series
    if series is None:
      e = """When field '%s' was accessed on instance '%s' belonging to 
      class '%s', the field instance returned 'None'! """
      msg = monoSpace(e % (fieldName, instance, owner))
      raise RuntimeError(msg)
    e = typeMsg(fieldName, series, QLineSeries)
    raise TypeError(e)

  def __str__(self) -> str:
    """Return the string representation of the field."""
    ownerName = self.getFieldOwner().__qualname__
    fieldName = self.getFieldName()
    return '%s.%s' % (ownerName, fieldName)

  def __repr__(self) -> str:
    """Return the string representation of the field."""
    clsName = self.__class__.__qualname__
    fieldName = self.getFieldName()
    return """%s = %s(...)""" % (fieldName, clsName)
