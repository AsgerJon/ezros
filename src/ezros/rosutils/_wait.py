"""WaitForIt allows interactive debugging."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
import subprocess

import subprocess
import tempfile
import os


class WaitForIt:
  def __init__(self):
    pass

  def __enter__(self):
    print("Interactive debugging session started. Enter ':q' to quit.")
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    print("Interactive debugging session ended.")

  def run_code(self):
    while True:
      with tempfile.NamedTemporaryFile(suffix=".py") as tmpfile:
        # Open vi for code editing
        subprocess.call(["vi", tmpfile.name])

        # Read the code from the temporary file
        with open(tmpfile.name, 'r') as f:
          code = f.read()

        if code.strip() == ":q":
          break

        # Execute the code
        try:
          result = subprocess.run(["python", tmpfile.name],
                                  capture_output=True,
                                  text=True)
          if result.stdout:
            print(result.stdout)
          if result.stderr:
            print(f"Error: {result.stderr}")
        except Exception as e:
          print(f"Exception occurred: {e}")
