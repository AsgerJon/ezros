"""ClassField provides a descriptor wrapper on custom classes. The owner
of such instances may explicitly specify accessor functions and in
particular the creator function. A typical use might be for the owner
class to explicitly provide a creator function using the 'CREATE'
decorator. Alternatively, the wrapped class may implement a suitable
method for explicit instance creation. Finally, the descriptor wrapper
does provide a fallback. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Any, Never

from vistutils.fields import AbstractField
from vistutils.parse import maybe
from vistutils.text import monoSpace


class ClassField(AbstractField):
  """ClassField provides a descriptor wrapper on custom classes. The owner
  of such instances may explicitly specify accessor functions and in
  particular the creator function. A typical use might be for the owner
  class to explicitly provide a creator function using the 'CREATE'
  decorator. Alternatively, the wrapped class may implement a suitable
  method for explicit instance creation. Finally, the descriptor wrapper
  does provide a fallback. """

  def __init__(self, cls: type, *args, **kwargs) -> None:
    creatorKwarg = kwargs.get('creator', None)
    creatorArg = None
    posArgs = []
    keyWordArgs = {k: v for (k, v) in kwargs.items() if k != 'creator'}
    for arg in args:
      if callable(arg) and creatorArg is None:
        creatorArg = arg
      else:
        posArgs.append(arg)
    self.__value_class__ = cls
    self.__init_creator__ = maybe(creatorKwarg, creatorArg, None)
    self.__decorated_creator__ = None
    self.__decorated_setter__ = None
    self.__source_creator__ = getattr(cls, '__instance_creator__', None)
    self.__positional_args__ = posArgs
    self.__keyword_args__ = keyWordArgs
    AbstractField.__init__(self, *posArgs, **keyWordArgs)

  @classmethod
  def getOwnerListName(cls) -> str:
    """Getter-function for the owner list name."""
    clsName = cls.__qualname__
    return '__%s_fields__' % clsName

  def __prepare_owner__(self, owner: type) -> None:
    """Prepare the owner for the ClassField."""
    ownerListName = self.getOwnerListName()
    fieldName = self._getFieldName()
    existing = getattr(owner, ownerListName, [])
    setattr(owner, ownerListName, [*existing, self])

  def _getPosArgs(self) -> list:
    """Getter-function for the positional arguments."""
    return self.__positional_args__

  def _getKwArgs(self) -> dict:
    """Getter-function for the keyword arguments."""
    return self.__keyword_args__

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Decorator for the creator function."""
    self.__decorated_creator__ = callMeMaybe
    return callMeMaybe

  def _getValueClass(self) -> type:
    """Getter-function for the value class."""
    return self.__value_class__

  def _inferCreator(self, ) -> Callable:
    """This method attempts to infer a suitable creator function."""
    owner = self._getFieldOwner()
    value = self._getValueClass()
    args, kwargs = self._getPosArgs(), self._getKwArgs()

    def createInstance() -> Any:
      """Inferred creator function"""
      return value(*args, **kwargs)

    return createInstance

  def _getCreator(self) -> Callable:
    """Getter-function for the creator function."""
    if self.__decorated_creator__ is not None:
      return self.__decorated_creator__
    if self.__init_creator__ is not None:
      return self.__init_creator__
    if self.__source_creator__ is not None:
      return self.__source_creator__
    return self._inferCreator()

  def _getSetter(self, ) -> Callable:
    """Getter-function for the setter function."""
    if self.__decorated_setter__ is not None:
      return self.__decorated_setter__
    cls = self._getValueClass()
    if hasattr(cls, '__field_setter__'):
      return getattr(cls, '__field_setter__')
    return self.__illegal_setter__

  def __illegal_setter__(self, *_) -> Never:
    """Illegal setter function"""
    e = """Could not locate setter function for '%s'!""" % self
    raise TypeError(monoSpace(e))

  def _createInstance(self, instance: Any, owner: Any, ) -> Any:
    """Create an instance of the value class."""
    pvtName = self._getPrivateName()
    creator = self._getCreator()
    value = creator()
    setattr(instance, pvtName, value)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter-function implementation"""
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self._createInstance(instance, owner)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any) -> None:
    """By default, instances of ClassField do not support assignment
    unless implemented by either the value class or the owner class. The
    value class may implements a function called '__field_setter__',
    and the owner may explicitly decorate a method to be used as a setter
    with fieldName.SET. The latter takes precedence."""
    setter = self._getSetter()
    if hasattr(setter, '__self__'):
      return setter(instance, value)
    return setter(self, instance, value)
