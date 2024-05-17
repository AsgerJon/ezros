"""I don't know why this file is necessary lmao"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

lol = """/home/AsgerJon/PycharmProjects/ezros/src/ezros/rosutils/"""
lol2 = os.path.join(lol, 'catkin_ws', 'devel', 'lib', 'python3.11')
lol3 = os.path.join(lol2, 'site-packages')

sys.path.append(lol3)
