"""The 'env' module provides package wide named values providing a vastly
superior alternative to using environment variables. Instead of hoping to
be in the right environment and that it has already been sourced, import
names from this module. Generally this module provides named values. If a
name require dynamic interaction at runtime, it should be implemented as
such."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._dir_names import getParentDir, changeDir

SITE_PACKAGES = getParentDir(__file__, 3)
EZ_ROOT = getParentDir(__file__, 2)
