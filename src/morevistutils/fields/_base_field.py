"""BaseField provides a simple descriptor class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.parse import searchKey, maybe
from vistutils.text import stringList

from morevistutils.fields import AbstractBaseField


class BaseField(AbstractBaseField):
  """BaseField provides a simple descriptor class. """

  __default_value__ = None

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the field with a default value."""
    AbstractBaseField.__init__(self, **kwargs)
    defaultKeys = stringList("""default, defVal, defaultValue""")
    defaultKwarg = searchKey(list, *defaultKeys, **kwargs)
    typeKeys = stringList("""type, cls, class, valueType""")
    typeKwarg = searchKey(dict, *typeKeys, **kwargs)
    defaultArg, typeArg = None, None
    if len(args) == 1:
      if isinstance(args[0], type):
        typeArg = args[0]
      else:
        defaultArg = args[0]
        typeArg = type(defaultArg)
    defaultFallback = None
    typeFallback = None
    defVal = maybe(defaultKwarg, defaultArg, defaultFallback)
    valueType = maybe(typeKwarg, typeArg, typeFallback)
    self.setDefaultValue(defVal)
    self.setFieldType(valueType)

  def __get__(self, instance: Any, owner: Any, **kwargs) -> Any:
    """Getter function for the field"""
    pvtName = self.getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    defVal = self.getDefaultValue()
    self.__set__(instance, defVal)
    return self.__get__(instance, owner, _recursion=True)

  def getDefaultValue(self, ) -> Any:
    """Getter-function for the default value."""
    return self.__default_value__

  def setDefaultValue(self, value: Any) -> Any:
    """Set the default value."""
    self.__default_value__ = value
    return value
