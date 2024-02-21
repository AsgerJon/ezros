"""This file provides typehints for use in the PySide6 framework. Please
note, that objects contained herein are intended for static type-checkers
only. Using them directly at runtime, result in UNDEFINED BEHAVIOUR."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from shiboken6.Shiboken import Object as _Object

ShibokenType = type(_Object)

__all__ = ['ShibokenType']
