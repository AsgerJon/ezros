"""WaitForIt allows interactive debugging."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
import subprocess

import subprocess
import sys
import tempfile
import os
import subprocess
import tempfile
import os

from vistutils.text import monoSpace


class WaitForIt:
  def __init__(self):
    pass

  def __enter__(self):
    print("Interactive debugging session started. Enter ':q' to quit.")
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    print("Interactive debugging session ended.")

  def run_code(self, obj: object = None):
    this = obj
    lines = []
    while True:
      os.system('cls' if os.name == 'nt' else 'clear')

      msg = """Enter code line by line. Enter commands by prepending colon
      (':'). The following commands are recognized: <br>
      :q - Quit the interactive debugging session. <br>
      :RUN - Run the code. <br>
      :SAVE [fileName] - Save the code to the file specified. <br>
      :LOAD [fileName] - Load the code from the file specified. <br>
      :CLEAR - Clear the code. <br>
      :vi - Open the code in the vi editor."""
      print(msg)
      for (i, line) in enumerate([*lines, '']):
        print('%03d ~ %s' % (i + 1, line))
      code = input(">>> ")
      if code.strip()[0] == ":":
        if code.strip() == ":q":
          break
        if code.strip() == ":CLEAR":
          lines = []
        if code.strip() == ":vi":
          raise NotImplementedError
        if code.strip() == ":RUN":
          os.system('python -c "%s"' % '\n'.join(lines))
      else:
        lines.append(code)
