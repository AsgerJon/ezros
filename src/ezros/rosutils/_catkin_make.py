"""This file runs the catkin built script. """
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from subprocess import Popen, PIPE

from icecream import ic
from vistutils.text import stringList

from ezros.env import getParentDir, changeDir
from ezros.rosutils import CATKIN_WS, getRosPackages

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
    """rm -rf build devel logs""",
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


def _realPythonPath() -> str:
  """Returns the path to the real python files."""
  return changeDir(_getThisConda(), 'lib', 'python3.11', 'site-packages')


def _moveCommands() -> list[str]:
  """Collects the commands to move the python files. """
  rosPackages = [os.path.basename(p) for p in getRosPackages()]
  realPythonPath = _realPythonPath()
  catkinPath = _catkinPythonPath()
  oldPythonFiles = [os.path.join(realPythonPath, p) for p in rosPackages]
  newPythonFiles = [os.path.join(catkinPath, p) for p in rosPackages]

  out = [_sourceCondaCommand(),
         _sourceMambaCommand(),
         _activateCondaCommand()]

  safetyWords = stringList("""lib, python3.11, site-packages""")
  for files in oldPythonFiles:
    if not os.path.isabs(files):
      e = """Received unexpected file path: '%s' when listing folders 
      for removal!"""
      raise RuntimeError(e % files)
    if not all(word in files for word in safetyWords):
      e = """Received unexpected file path: '%s' when listing folders 
      for removal!"""
      raise RuntimeError(e % files)
    out.append("""rm -rf %s""" % files)

  for files in newPythonFiles:
    out.append("""cp -rf %s %s""" % (files, realPythonPath))

  for files in out:
    print(out)

  return out


def _catkinPythonPath() -> str:
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
  process = Popen(['/bin/zsh'],
                  stdin=PIPE,
                  stdout=PIPE,
                  stderr=PIPE,
                  text=True)
  try:
    commands = '\n'.join(_moveCommands())
  except Exception as exception:
    raise SystemExit from exception
  stdout, stderr = process.communicate(commands)
  if process.returncode:
    raise RuntimeError(stderr)
  print(stdout)
