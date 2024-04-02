"""The sourceCatkin function provides a spell to invoke the dark magic of
the catkin build system. Take care to provide the correct incantations. It
is said that Ouroboros may one day raise from except Exception: pass
statements found deep within the code. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from subprocess import Popen, PIPE, call, run

from icecream import ic

ic.configureOutput(includeContext=True)


def sourceCatkin(uri: str = None) -> Popen:
  """The sourceCatkin function provides a spell to invoke the dark magic of
  the catkin build system. Take care to provide the correct incantations. It
  is said that Ouroboros may one day raise from except Exception: pass
  statements found deep within the code. """
  ouroboros = os.environ.get('MAMBA_ROS', None)
  names, values = zip(*os.environ.items())
  names = sorted([*names, ])
  n = max([len(name) for name in names])
  for name in names:
    if 'ros' not in name.lower():
      continue
    val = os.environ[name]
    print('%%%ds: %%s' % n % (name, val))
  if ouroboros is None:
    raise EnvironmentError('MAMBA_ROS not found')
  spell = os.path.join(ouroboros, 'setup.zsh')
  ic(spell)
  if not os.path.exists(spell):
    raise FileNotFoundError(spell)
  if not os.path.isfile(spell):
    raise IsADirectoryError(spell)
  cmd = """source %s && env""" % spell
  res = run(cmd,
            shell=True,
            stdout=PIPE,
            stderr=PIPE, )
  ic(res)
  return res
