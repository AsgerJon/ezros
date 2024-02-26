"""Signature instances are frozen lists of types."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from morevistutils import resolveType, typeCast, resolveInstance


class Signature:
  """Signature instances are frozen lists of types."""

  __iter_contents__ = None

  @classmethod
  def fromTypes(cls, *args) -> Signature:
    """Create a new Signature instance."""
    types, errors = cls.parse(*args)
    self = cls(*types)
    self._errors = errors
    return self

  @classmethod
  def parse(cls, *args) -> tuple[list[type], list[dict]]:
    """Parse the args into a Signature instance."""
    types = []
    errors = []
    for arg in args:
      if isinstance(arg, type):
        types.append(arg)
      elif isinstance(arg, str):
        try:
          type_ = resolveType(arg)
          types.append(type_)
        except NameError as nameError:
          sample = 'is not assigned to a type in globals() or'
          if sample in str(nameError):
            errors.append({'name': arg, 'error': nameError})
            continue
          raise nameError
      elif isinstance(arg, tuple):
        arg = [*arg, ]
      elif isinstance(arg, Signature):
        arg = [*arg, ]
      elif isinstance(arg, list):
        parsed = cls.parse(*arg)
        types.extend(parsed[0])
        errors.extend(parsed[1])
    return types, errors

  def __init__(self, *args) -> None:
    types = []
    errors = []
    self._types, self._errors = self.parse(*args)

  def __iter__(self) -> Signature:
    """Implementation of the iterator protocol. This method lists the
    types making up this signature."""
    self.__iter_contents__ = [type_ for type_ in self._types]
    return self

  def __next__(self) -> type:
    """Implementation of the next protocol. This method returns the next
    type in the signature."""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration

  def __len__(self) -> int:
    """Implementation of the length protocol. This method returns the
    number of types in this signature."""
    return len(self._types)

  def __bool__(self) -> bool:
    """Implementation of the truth protocol. This method returns True if
    there are any types in this signature."""
    return True if self._types else False

  def __eq__(self, other: Signature) -> bool:
    """Implementation of the equality protocol. This method returns True
    if the other signature is equal to this one."""
    if isinstance(other, Signature):
      if self and other:
        for (mine, yours) in zip(self, other):
          if mine != yours:
            return False
        return True
      return False
    try:
      self == Signature(*other)
    except TypeError as typeError:
      sample = 'object is not iterable'
      if sample in str(typeError):
        try:
          self == Signature(other)
        except Exception as exception:
          raise exception from typeError
      raise typeError

  def __contains__(self, *others: Any) -> bool:
    """Implementation of the containment protocol. This method returns
    True if the other type is in this signature."""
    if len(others) == 1:
      other = others[0]
      if isinstance(other, type):
        return True if other in self._types else False
      if isinstance(other, Signature):
        return True if self == other else False
    for (type_, other) in zip(self, others):
      test = typeCast(other, type_)
      if not test[0]:
        return False


class Sample:
  """Instances of sample are tuples of objects matching the instance
  signature"""

  __inner_signature__ = None

  def __init__(self, *args) -> None:
    """Create a new Sample instance."""
    if len(args) > 1 and any([isinstance(arg, Signature) for arg in args]):
      e = """Instances of sample class based on a signature must not be 
      instantiated with more than that signature as argument!"""
      raise ValueError(e)
    if isinstance(args[0], Signature):
      self._signature = args[0]
      self._objects = self.getDefault()
    self._signature = Signature(*args)
    self._objects = [*args, ]

  def getDefault(self) -> list:
    """Get the default objects for the signature."""
    return [resolveInstance(type_) for type_ in self._signature]
