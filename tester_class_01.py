"""Tester class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

#
# class Meta(type):
#   """Metaclass for Tester"""
#
#   @classmethod
#   def __prepare__(mcls, name: str, bases: tuple, **kwargs) -> dict:
#     """Prepare the class"""
#     print(mcls.__qualname__, mcls.__prepare__.__name__)
#     return {'__name__': name, '__bases__': bases, '__kwargs__': kwargs}
#
#   def __new__(mcls,
#               name: str,
#               bases: tuple,
#               namespace: dict,
#               **kwargs) -> Meta:
#     """Create the class"""
#     print(mcls.__qualname__, mcls.__new__.__name__)
#     return super().__new__(mcls, name, bases, namespace)
#
#   def __init__(cls,
#                name: str,
#                bases: tuple,
#                namespace: dict,
#                **kwargs) -> None:
#     """Initialize the class"""
#     mcls = cls.__class__
#     print(mcls.__qualname__, cls.__qualname__, '__init__')
#     super().__init__(name, bases, namespace)
#
#   def __call__(cls, *args, **kwargs) -> Any:
#     """Call the class"""
#     print(cls.__qualname__, cls.__call__.__name__)
#     return super().__call__(*args, **kwargs)
