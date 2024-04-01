"""The sourceCatkin function provides a spell to invoke the dark magic of
the catkin build system. Take care to provide the correct incantations. It
is said that Ouroboros may one day raise from except Exception: pass
statements found deep within the code. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from subprocess import Popen, PIPE

from icecream import ic

ic.configureOutput(includeContext=True)


def sourceCatkin() -> Popen:
  """The sourceCatkin function provides a spell to invoke the dark magic of
  the catkin build system. Take care to provide the correct incantations. It
  is said that Ouroboros may one day raise from except Exception: pass
  statements found deep within the code. """
  ouroboros = os.environ.get('MAMBA_ROS', None)
  names, values = zip(*os.environ.items())
  n = max([len(name) for name in names])
  for (key, val) in os.environ.items():
    if key == 'MAMBA_ROS':
      print(37 * '*')
    print('%%%ds: %%s' % n % (key, type(val)))
    if key == 'MAMBA_ROS':
      print(37 * '*')
  if ouroboros is None:
    raise EnvironmentError('MAMBA_ROS not found')
  spell = os.path.join(ouroboros, 'setup.zsh')
  if not os.path.exists(spell):
    raise FileNotFoundError(spell)
  if not os.path.isfile(spell):
    raise IsADirectoryError(spell)
  cmd = """source %s && env""" % spell
  res = Popen(cmd,
              shell=True,
              executable='/bin/zsh',
              stdout=PIPE,
              stderr=PIPE,
              text=True)
  if res.returncode:
    raise RuntimeError(res.stderr.read())
  return res
