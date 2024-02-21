"""Module for converting time domain signals to frequency domain signals."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
import numpy as np


def linearInterpolation(t, x, numPoints):
  """
  Linearly interpolate the data onto a uniform grid.

  Args:
    t: Time data (non-uniformly sampled).
    x: Position data.
    numPoints: Number of points in the uniform grid.

  Returns:
    tUniform: Uniformly sampled time data.
    xUniform: Interpolated position data.
  """
  tUniform = np.linspace(t.min(), t.max(), numPoints)
  xUniform = np.interp(tUniform, t, x)
  return tUniform, xUniform


def computeFft(t: np.ndarray, x: np.ndarray) -> tuple[
  np.ndarray, np.ndarray]:
  """
  Compute the FFT of non-uniformly sampled data.

  Args:
    t: Time data (non-uniformly sampled).
    x: Position data.

  Returns:
    f: Frequency data.
    r: Intensity data.
  """
  # Interpolate the data onto a uniform grid
  tUniform, xUniform = linearInterpolation(t, x, len(t))

  # Compute the time step (now uniform)
  dt = tUniform[1] - tUniform[0]

  # Compute the FFT of the interpolated position data
  r = np.fft.fft(xUniform)

  # Compute the corresponding frequency values
  f = np.fft.fftfreq(len(xUniform), dt)

  # Since the FFT output is complex, we'll take the absolute value to get
  # the intensity
  r = np.abs(r)

  # Return the frequency and intensity arrays
  return f, r
