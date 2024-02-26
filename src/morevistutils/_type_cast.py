"""The typeCast function receives a value and a type and attempts to cast
the value to the type. If the type is a custom class, the function will
pass the value to the class constructor and return the result or allow the
error to propagate. Common types are handled individually."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import json
from typing import Any


def stringNumber(value: str) -> Any:
  """Attempts to find the value of the number represented by the string."""
  digs = []
  commaPlace = []
  sign = []

  digits = {'%s' % i: i for i in range(10)}
  c = 0
  for (i, char) in enumerate(value):
    if char in digits:
      digs.append(digits[char])
    elif char == '.':
      commaPlace.append(len(digs))
    elif not digs and char in ['+', '-']:
      sign.append(char)

  if len(commaPlace) > 1:
    raise ValueError('The string contains more than one decimal point!')
  if len(sign) > 1:
    raise ValueError('The string contains more than one sign!')
  if not digs:
    raise ValueError('The string contains no digits!')
  if not commaPlace:
    factor = None
  else:
    factor = 10 ** (len(digs) - commaPlace[0] - 1)
  out = 0
  ten = 1
  while digs:
    out += digs.pop() * ten
    ten *= 10
  out = int(out) if '-' not in sign else -int(out)
  if factor is None:
    return out
  return out / factor


def _typeCast(value: Any, type_: type) -> Any:
  """The typeCast function receives a value and a type and attempts to cast
  the value to the type. If the type is a custom class, the function will
  pass the value to the class constructor. """
  if value.__class__ is type_ or type_ is object or isinstance(value, type_):
    return value
  if type_ in [int, float, complex] and isinstance(value, str):
    sample = 'The string contains'
    try:  # Try to cast the string to a number
      value = stringNumber(value)
    except ValueError as valueError:
      if sample not in str(valueError):
        raise valueError
      e = """Unable to cast '%s' as a number!""" % value
      raise TypeError(e) from valueError
    if type_ is complex:
      return value + 0j
    if type_ is float:
      return float(value)
    if type_ is int:
      if (value - round(value)) ** 2 < 1e-10:
        return int(round(value))
      e = """The value '%s' is not an integer!""" % value
      raise TypeError(e)
  if type_ is str:
    return str(value)
  if type_ is bool:
    return True if value else False
  if type_ is list:
    try:
      return [*value, ]
    except Exception as exception:
      sample = 'is not iterable'
      if sample in str(exception):
        e = """Unable to cast '%s' as a list!""" % value
        raise TypeError(e) from exception
      raise exception
  if type_ is tuple:
    try:
      return (*value,)
    except Exception as exception:
      sample = 'is not iterable'
      if sample in str(exception):
        e = """Unable to cast '%s' as a tuple!""" % value
        raise TypeError(e) from exception
      raise exception
  if type_ is dict and isinstance(value, str):
    try:
      value = json.loads(value)
      return value
    except Exception as exception:
      e = """Unable to cast '%s' as a dictionary!""" % value
      raise TypeError(e) from exception
  try:
    self = type.__call__(type_, value)
    return self
  except Exception as exception:
    e = """Unable to cast '%s' as a '%s'!""" % (value, type_.__qualname__)
    raise TypeError(e) from exception


def typeCast(value: Any, type_: type, **kwargs) -> Any:
  """The typeCast function receives a value and a type and attempts to cast
  the value to the type. If the type is a custom class, the function will
  pass the value to the class constructor and return the result or allow the
  error to propagate. Common types are handled individually."""
  try:
    return True, _typeCast(value, type_)
  except TypeError as typeError:
    if kwargs.get('strict', False):
      raise typeError
    return False, None
