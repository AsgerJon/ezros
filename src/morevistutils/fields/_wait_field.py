"""WaitField provides a descriptor class for fields that create the value
on demand rather than at the field instantiation time. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any

from icecream import ic
from vistutils.waitaminute import typeMsg

from morevistutils.fields import AbstractBaseField, FlexField, BaseField


class WaitField(FlexField):
  """WaitField provides a descriptor class for fields that create the value
  on demand rather than at the field instantiation time. """

  __explicit_creator__ = None

  posArgs = BaseField(list)
  keyArgs = BaseField(dict)

  def __init__(self, cls: type, *args, **kwargs) -> None:
    """Initializes the field with a function that will be called to create
    the value when it is first accessed. """
    ic(args)
    if not isinstance(cls, type):
      args = [cls, *args]
      cls = None
    func, leftArgs = None, []
    for arg in args:
      if isinstance(arg, type) and cls is None:
        cls = arg
      elif callable(arg) and func is None:
        func = getattr(arg, '__func__', arg)
      else:
        leftArgs.append(arg)
    if cls is None:
      e = """Instances of WaitField must be instantiated with their 
      intended type as a positional argument!"""
      raise ValueError(e)

    AbstractBaseField.__init__(self, *leftArgs, **kwargs)
    self.setFieldType(cls)
    self.posArgs = [*leftArgs, ]
    self.keyArgs = {**kwargs, }

  def setCreator(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for explicit creator"""
    if self.__explicit_creator__ is not None:
      e = 'Creator function already set.'
      raise AttributeError(e)
    self.__explicit_creator__ = callMeMaybe
    return callMeMaybe

  def getCreator(self) -> Callable:
    """Getter-function for explicit creator falling back to field type"""
    if self.__explicit_creator__ is None:
      return self.getFieldType()
    return self.__explicit_creator__

  def instantiate(self, instance: Any, owner: type) -> None:
    """This method instantiates the value of the field. """
    pvtName = self.getPrivateName()
    creator = self.getCreator()
    value = creator(*self.posArgs, **self.keyArgs)
    setattr(instance, pvtName, value)

  def __get__(self, instance: Any, owner: Any, **kwargs) -> Any:
    """Getter function for the field"""
    pvtName = self.getPrivateName()
    existing = getattr(instance, pvtName, None)
    fieldType = self.getFieldType()
    if existing is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.instantiate(instance, owner)
      return self.__get__(instance, owner, _recursion=True)
    if isinstance(existing, fieldType):
      return existing
    e = typeMsg('existing', existing, fieldType)
    raise TypeError(e)

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """This decorator sets the creator function for the field. """
    return self.setCreator(callMeMaybe)
