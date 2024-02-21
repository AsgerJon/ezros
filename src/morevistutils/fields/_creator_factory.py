"""The creatorFactory function takes a type as input and returns a creator
function for that type. The first positional argument is the type in
question, but the remaining arguments are passed to the inner creator
function. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable


def creatorFactory(cls: type, *args, **kwargs) -> Callable:
  """The creatorFactory function takes a type as input and returns a creator
  function for that type. The first positional argument is the type in
  question, but the remaining arguments are passed to the inner creator
  function. """

  baseArgs, baseKwargs = args, kwargs

  def creator(*posArgs, **keyWordArgs) -> type:
    """Automatically generated creator function for type: '%s'"""
    combinedArgs = [*baseArgs, *posArgs]
    combinedKwargs = {**baseKwargs, **keyWordArgs}
    try:
      return type.__call__(cls, *combinedArgs, **combinedKwargs)
    except TypeError as typeError:
      if 'takes at most 1' in str(typeError):
        return type.__call__(
          cls, *[*combinedArgs, None][:1], **combinedKwargs)
      try:
        return type.__call__(cls, )
      except Exception as exception:
        raise exception from typeError

  return creator
