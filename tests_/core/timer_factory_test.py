"""Unit tests for the TimerFactory class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import unittest
from PySide6.QtCore import Qt

from _dep.morevistside.core import Precise
from _dep.morevistside.core import _parseArgs


class TestParseTimer(unittest.TestCase):
  def test_parse_timer_raises_value_error(self):
    with self.assertRaises(ValueError):
      _parseArgs(singleShot=True, repeat=False)

  def test_parse_timer_default_values(self):
    timer_properties = _parseArgs()
    self.assertIsInstance(timer_properties, dict)
    self.assertEqual(timer_properties['epoch'], 100)
    self.assertEqual(timer_properties['shot'], False)
    self.assertEqual(timer_properties['timerType'], Precise)

  def test_parse_timer_kwargs(self):
    timer_properties = _parseArgs(epoch=200,
                                  shot=True,
                                  timerType=Qt.CoarseTimer)
    self.assertIsInstance(timer_properties, dict)
    self.assertEqual(timer_properties['epoch'], 200)
    self.assertEqual(timer_properties['shot'], True)
    self.assertEqual(timer_properties['timerType'], Qt.CoarseTimer)

  def test_parse_timer_timerType(self):
    timer_properties = _parseArgs(timerType=Qt.VeryCoarseTimer)
    self.assertIsInstance(timer_properties, dict)
    self.assertEqual(timer_properties['timerType'], Qt.VeryCoarseTimer)
