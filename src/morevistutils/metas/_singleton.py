"""Singleton provides a metaclass for deriving singleton classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never, Callable

from icecream import ic
from vistutils.metas import AbstractMetaclass
from vistutils.text import monoSpace
from vistutils.waitaminute import EffortException

from morevistutils.fields import Later


class _Field:

  def __init__(self, initialValue: Any = None) -> None:
    self.__field_name__ = None
    self.__field_owner__ = None
    self.__initial_value__ = initialValue

  def __set_name__(self, owner, name) -> None:
    """This method sets the name and owner of the field and is
    automatically called when the owner class is created. """
    self.__field_name__ = name
    self.__field_owner__ = owner

  def _getPrivateName(self, ) -> str:
    """Getter function for the private name"""
    return '_%s' % self.__field_name__

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    if instance is None:
      if self.__initial_value__ is not None:
        return self.__initial_value__
      e = """To be accessed through the class, the field must have an
      initial value!"""
      raise AttributeError(e)
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    if self.__initial_value__ is None:
      e = """The field was created without initial value and accessed 
      before any value were set!"""
      raise AttributeError(e)
    self.__set__(instance, self.__initial_value__)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: Any, value: Any) -> None:
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, value)

  def __delete__(self, instance: Any) -> None:
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return delattr(instance, pvtName)
    e = """The field named: '%s' does not exist in the instance: '%s'!"""
    raise AttributeError(monoSpace(e % (self.__field_name__, instance)))


class _Args:

  def __init__(self, ) -> None:
    self.__field_name__ = None
    self.__field_owner__ = None

  def __set_name__(self, owner, name) -> None:
    """This method sets the name and owner of the field and is
    automatically called when the owner class is created. """
    self.__field_name__ = name
    self.__field_owner__ = owner

  def __get__(self, instance: type, owner: type, **kwargs) -> Any:
    argsName = self._getArgsName()
    kwargsName = self._getKwargsName()
    ownerArgs = getattr(instance, argsName, None)
    ownerKwargs = getattr(instance, kwargsName, None)
    if ownerArgs is None and ownerKwargs is None:
      return self
    if ownerArgs is None or ownerKwargs is None:
      e = """Unexpected state! Either both args and kwargs should be None 
      or neither!"""
      raise AttributeError(e)
    return [*ownerArgs, ], {**ownerKwargs, }

  def __set__(self, instance: type, value: Any) -> None:
    """LMAO, this actually works, as long as the field is owned by the
    metaclass!"""
    args, kwargs = value
    self.setValue(instance, *args, **kwargs)

  def __delete__(self, *_) -> Never:
    e = """The _Args descriptor is not intended for setting!"""
    raise TypeError(e)

  def _getArgsName(self, ) -> str:
    """Getter function for the private name"""
    return '__%s_args__' % self.__field_name__

  def _getKwargsName(self, ) -> str:
    """Getter function for the private name"""
    return '__%s_kwargs__' % self.__field_name__

  def setValue(self, owner: type, *args, **kwargs) -> None:
    """Setter function for the value of the descriptor. Please note,
    that descriptors accessed through the class will not invoke their
    '__set__' method, but will instead be overwritten themselves.

    For example, if field is a descriptor of class Field owned by class
    Owner, then:
    class Field:
      def __set__(self, instance: Any, value: Any) -> None:
        ...

      def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
        return 69

    class Owner:
      field = Field()

    Owner.field = 420 results in:
    setattr(Owner, 'field', 420)
    So after this, the name 'field' is populated by the integer 420,
    but by the field instance. This contradicts what would be expected:
    Field.__set__(field, Owner, 420)
    The inconsistency comes from the fact that the __get__ method does not
    behave in this way when accessed through the owner. In that case:
    isinstance(Owner.field, int)  # True
    Is in fact:
    isinstance(Field.__get__(field, None, Owner), int)  # or:
    isinstance(69, int)  # True

    It is for this reason that when the private variable on the owner class
    is not yet set, the __get__ method returns the instance itself. This
    allows the carelessness implemented here.

    EDIT: omg, when the descriptor class is instantiated in the body of
    the metaclass, it shifts so that the owner is the metaclass and
    instance is the class. This is so cool!"""
    argsName = self._getArgsName()
    kwargsName = self._getKwargsName()
    setattr(owner, argsName, args)
    setattr(owner, kwargsName, kwargs)


class SingletonMeta(AbstractMetaclass):
  """Singleton metaclass for deriving singleton classes."""

  __call_args__ = _Args()
  __singleton_instance__ = Later()
  __decorator_class__ = None

  @__singleton_instance__.CREATE
  def __explicit_creator__(cls, *__, **_) -> Any:
    """This method actually creates the instance. """
    args, kwargs = cls.__call_args__
    return AbstractMetaclass.__call__(cls, *args, **kwargs)

  def __init__(cls, *args, **kwargs) -> None:
    """Creates the singleton instance. """

  def __call__(cls, *args, **kwargs) -> Any:
    """This method returns the singleton instance of the class. If the
    instance does not exist, it is created. """
    if cls is SingletonMeta.__decorator_class__:
      if args:
        cls_ = args[0]
        if isinstance(cls_, type):
          return cls()(cls_)
    cls.__call_args__ = args, kwargs
    return cls.__singleton_instance__

  @classmethod
  def decoratify(mcls, cls: SingletonMeta) -> SingletonMeta:
    """This method decorates the class that be used to decorate other
    classes. """
    if mcls.__decorator_class__ is not None:
      e = """The decorator class has already been set!"""
      raise AttributeError(e)
    if cls.__class__ is not SingletonMeta:
      e = """The class to become the decorator must be derived from the 
      SingletonMeta metaclass!"""
      raise TypeError(e)

    def _decorate(self, cls_: type, *__, **_) -> type:
      """This function decorates the class with the decorator class. """
      kwargs = dict(_root='f... da police!')
      return SingletonMeta(cls_.__qualname__, (cls_,), {}, **kwargs)

    setattr(cls, '__call__', _decorate)
    mcls.__decorator_class__ = cls
    return cls


@SingletonMeta.decoratify
class Singleton(metaclass=SingletonMeta):
  """Singleton is a baseclass for deriving singleton classes. This in
  between class exposes the metaclass 'SingletonMeta' ensuring that
  subclasses are singletons. """

  def __init__(self, *args, **kwargs) -> None:
    """This method is here so that the static type checker doesn't
    complain. It still complains, but now it's because a subclass
    neglected a super call. """

  def __call__(self, *__, **_) -> type:
    """I'm just here to stop the static typechecker from freaking out
    lmao!"""

  def __init_subclass__(cls, /, **kwargs) -> None:
    """Fine, I'll take those keyword arguments so that
    object.__init_subclass__ doesn't throw an error before not doing
    anything anyway."""
    return object.__init_subclass__()
