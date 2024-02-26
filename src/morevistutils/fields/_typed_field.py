"""TypedField provides a strongly typed descriptor class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField, BaseField


class TypedField(BaseField):
  """TypedField provides a strongly typed descriptor class."""

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the field with a type."""
    AbstractBaseField.__init__(self, *args, **kwargs)
    type_, defVal = None, None
    if not args:
      raise ValueError('No arguments given!')
    if len(args) == 1:
      if isinstance(args[0], type):
        type_ = args[0]
      else:
        defVal = args[0]
        type_ = type(defVal)
    if len(args) > 1:
      if isinstance(args[0], type) and not isinstance(args[1], type):
        type_, defVal = [*args, None][:2]
      elif not isinstance(args[0], type) and isinstance(args[1], type):
        defVal, type_ = [*args, None][:2]
      elif isinstance(args[0], type) and isinstance(args[1], type):
        e = """Unsupported signature: ('%s', '%s')""" % (type, type)
        raise TypeError(e)
      elif not isinstance(args[0], type) and not isinstance(args[1], type):
        e = """Unsupported signature: ('%s', '%s')""" % (object, object)
        raise TypeError(e)
      if not isinstance(defVal, type_):
        e = typeMsg('defVal', defVal, type_)
        raise TypeError(e)
    if type_ is None:
      e = """No type given!"""
      raise ValueError(e)
    BaseField.__init__(self, *args, **kwargs)

  def __get__(self, instance: Any, owner: Any, **kwargs) -> Any:
    """Getter function for the field"""
    value = BaseField.__get__(self, instance, owner)
    if isinstance(value, self.getFieldType()):
      return value
    e = typeMsg('value', value, self.getFieldType())
    raise TypeError(e)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter function for the field"""
    if not isinstance(value, self.getFieldType()):
      e = typeMsg('value', value, self.getFieldType())
      raise TypeError(e)
    return BaseField.__set__(self, instance, value)

  def __delete__(self, instance: Any) -> None:
    """Deleter function for the field"""
    return BaseField.__delete__(self, instance)
