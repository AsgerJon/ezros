"""TypedField provides a baseclass for strongly typed descriptors"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.waitaminute import typeMsg

from morevistutils import AbstractDescriptor


class TypedField(AbstractDescriptor):
  """FlexField provides a descriptor class for mutable types. When first
  accessed on an instance, an object of the mutable type is created and
  assigned to the instance. The setter and deleter will then augment or
  reset the object as appropriate."""

  __default_value__ = None

  @classmethod
  def _parse(cls, *args) -> tuple[type, object]:
    fieldType, defaultValue = None, None
    if len(args) == 1:
      if isinstance(args[0], type):
        fieldType = args[0]
      else:
        fieldType = type(args[0])
        defaultValue = args[0]
    elif len(args) == 2:
      if isinstance(args[0], type):
        fieldType = args[0]
        defaultValue = args[1]
        if not isinstance(defaultValue, fieldType):
          e = typeMsg('defaultValue', defaultValue, fieldType)
          raise TypeError(e)
      elif isinstance(args[1], type):
        return cls._parse(args[1], args[0])
    else:
      for arg in args:
        if isinstance(arg, type) and fieldType is None:
          fieldType = arg
          break
      else:
        e = """No type found in the arguments!"""
        raise ValueError(e)
      if isinstance(fieldType, type):
        for arg in args:
          if isinstance(arg, fieldType) and defaultValue is None:
            defaultValue = arg
            break
      else:
        e = typeMsg('fieldType', fieldType, type)
        raise TypeError(e)
    return fieldType, defaultValue

  def __init__(self, *args) -> None:
    fieldType, defaultValue = self._parse(*args)
    self._setFieldType(fieldType)
    self.__default_value__ = defaultValue

  def _getDefaultValue(self) -> Any:
    """Returns the default value."""
    if self.__default_value__ is not None:
      return self._typeGuard(self.__default_value__)
    e = """This instance of TypedField provides no default value!"""
    raise ValueError(e)

  def _instantiate(self, instance: object, owner: type = None) -> None:
    """Instantiates the field"""
    try:
      defVal = self._getDefaultValue()
      return setattr(instance, self._getPrivateName(), defVal)
    except ValueError as valueError:
      try:
        AbstractDescriptor._instantiate(self, instance, owner)
      except AttributeError as attributeError:
        raise valueError from attributeError
