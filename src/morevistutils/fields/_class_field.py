"""ClassField provides a descriptor class with deferred instance creation."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never, Callable, Optional

from vistutils.parse import maybe
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from morevistutils.fields import creatorFactory, AbstractField


class ClassField(AbstractField):
  """ClassField provides a descriptor class with deferred instance
  creation."""

  def __init__(self, fieldType: type, *args, **kwargs) -> None:
    """Initializes the ClassField"""
    self.__field_type__ = fieldType
    self.__auto_creator__ = creatorFactory(fieldType, *args, **kwargs)
    self.__explicit_creator__ = None
    self.__positional_args__ = [*args, ]
    self.__keyword_args__ = {**kwargs, }

  def instantiate(self, instance: Any, owner: type) -> None:
    """This method first creates an object of the field type as
    appropriate for the given instance and owner. This object is then
    placed at the private name of this field on the instance. By default,
    the owner itself will not be given a dedicated object, instead it will
    return the field type itself."""
    if instance is None:
      e = """Cannot create instance for the owner itself as that would 
      overwrite the descriptor itself."""
      raise RuntimeError(e)
    try:
      obj = self.__get__(instance, owner, _recursion=True)
    except RecursionError as recursionError:
      creator = self.getCreator()
      obj = creator(instance, owner)
      return setattr(instance, pvtName, obj)
    e = """The instance: '%s' already had an object instantiated at field 
    named: '%s': '%s'!"""
    thisName = str(instance)
    item = str(obj)
    raise AttributeError(monoSpace(e % (thisName, self.fieldName, item)))

  def getCreator(self, ) -> Callable:
    """Getter-function for the creator function. """
    creator = maybe(self.__explicit_creator__, self.__auto_creator__)
    if callable(creator):
      return creator
    e = typeMsg('creator', creator, Callable)
    raise TypeError(e)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter for the field"""
    if instance is None:
      return self
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      out = getattr(instance, pvtName)
      if isinstance(out, self.fieldType):
        return out
      e = typeMsg(pvtName, out, self.fieldType)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.instantiate(instance, owner)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any) -> Never:
    """Subclasses must implement setter for it to be available"""
    raise NotImplementedError

  def __delete__(self, instance: Any) -> Never:
    """Subclasses must implement deleter for it to be available"""
    raise NotImplementedError

  def __prepare_owner__(self, owner: type) -> type:
    """When this method is invoked, all fields on the owner class are
    certain to be defined. """
    ownerListName = self._getOwnerFieldListName()
    if not hasattr(owner, ownerListName):
      setattr(owner, ownerListName, [self])
      return owner
    existing = getattr(owner, ownerListName, )
    setattr(owner, ownerListName, [*existing, self])
    setattr(owner, '__prepared_for_%s__' % self.__class__.__qualname__, True)
    __existing_init__ = getattr(owner, '__init__', )

    def wrapInit(this, *args, **kwargs) -> None:
      """Wrapper on the __init__ method on the owner that."""
      __existing_init__(this, *args, **kwargs)
      self.instantiate(this, self.fieldOwner)

    setattr(owner, '__init__', wrapInit)
    return owner

  def _setExplicitCreator(self, callMeMaybe: Callable) -> Callable:
    """Run-once setter-function for the explicit creator-function"""
    if self.__explicit_creator__ is not None:
      raise TypeError
    self.__explicit_creator__ = callMeMaybe
    return callMeMaybe

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Decorates the callable as the creator function for this instance.
    If defined, this takes precedence. """
    return self._setExplicitCreator(callMeMaybe)

  def __lshift__(self, other: Callable) -> Callable:
    """Decorates the callable as the creator function for this instance.
    If defined, this takes precedence. """
    if callable(other):
      return self._setExplicitCreator(other)
    return NotImplemented
