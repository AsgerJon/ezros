"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
#
# from typing import Callable
#
# from icecream import ic
#
# from tester_class_01 import Meta
#
# ic.configureOutput(includeContext=True)
#
#
# class Descriptor:
#   """Descriptor"""
#   print('Creating descriptor class')
#
#   def __init__(self, ) -> None:
#     print('Instantiating descriptor class')
#
#   def __set_name__(self, owner, name) -> None:
#     print(self.__class__.__qualname__, self.__set_name__.__name__)
#
#   def __get__(self, instance, owner) -> None:
#     print(self.__class__.__qualname__, self.__get__.__name__)
#
#   def this(self, callMeMaybe: Callable) -> Callable:
#     """Decorates the callable"""
#     print("""Decorating: '%s'""" % callMeMaybe.__qualname__)
#     print(callMeMaybe)
#     notations = getattr(callMeMaybe, '__qualname__', 'No annotations')
#     print("""Having annotations: '%s'""" % notations)
#     callMeMaybe(1, 2, '3')
#     return callMeMaybe
