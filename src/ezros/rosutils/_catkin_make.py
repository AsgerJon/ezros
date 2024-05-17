"""This file runs the catkin built script. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from subprocess import Popen, PIPE

from icecream import ic

from ezros.env import getParentDir, changeDir
from ezros.rosutils import CATKIN_WS

ic.configureOutput(includeContext=True)


def _getThisConda() -> str:
  """Returns the path to the running conda environment. """
  return os.environ['CONDA_PREFIX']


def _getBaseConda() -> str:
  """Returns the base conda path."""
  return getParentDir(_getThisConda(), 2)


def _sourceCondaCommand() -> str:
  """Returns the conda.sh file path."""
  return """source %s/etc/profile.d/conda.sh""" % _getBaseConda()


def _sourceMambaCommand() -> str:
  """Returns the mamba.sh file path."""
  return """source %s/etc/profile.d/mamba.sh""" % _getBaseConda()


def _activateCondaCommand() -> str:
  """Returns the command to activate the conda environment. """
  return """mamba activate %s""" % os.path.basename(_getThisConda())


def _sourceBash() -> str:
  """Returns the command to source the bash file. """
  return 'source %s' % changeDir(CATKIN_WS, 'devel', 'setup.bash')


def _collectCommands() -> list[str]:
  """Collects the commands to run the catkin_make script. """
  return [
    _sourceCondaCommand(),
    _sourceMambaCommand(),
    _activateCondaCommand(),
    """cd %s""" % CATKIN_WS,
    """rm -rf build devel""",
    """catkin build""",
    """exit"""
  ]


def _postCommands() -> list[str]:
  """Collects the commands to run the catkin_make script. """
  return [
    _sourceCondaCommand(),
    _sourceMambaCommand(),
    _activateCondaCommand(),
    """cd %s""" % changeDir(CATKIN_WS, 'devel'),
    _sourceBash(),
    """exit"""
  ]


def _getPythonPath() -> str:
  """Returns the path to the newly generated python files."""
  return changeDir(CATKIN_WS, 'devel', 'lib', 'python3.11', 'site-packages')


def runCatkinMake() -> None:
  """Runs the catkin make script. """
  process = Popen(['/bin/zsh'],
                  stdin=PIPE,
                  stdout=PIPE,
                  stderr=PIPE,
                  text=True)
  commands = '\n'.join(_collectCommands())
  ic(changeDir(CATKIN_WS, 'devel', 'setup.bash'))
  stdout, stderr = process.communicate(commands)
  if process.returncode:
    raise RuntimeError(stderr)
  print(stdout)
  commands = '\n'.join(_postCommands())
  process = Popen(['/bin/zsh'],
                  stdin=PIPE,
                  stdout=PIPE,
                  stderr=PIPE,
                  text=True)
  stdout, stderr = process.communicate(commands)
  if process.returncode:
    raise RuntimeError(stderr)
  print(stdout)
