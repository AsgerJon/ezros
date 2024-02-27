"""PubMain provides the business logic of the main application window"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ezros.pub import PubLayoutWindow


class PubMain(PubLayoutWindow):
  """The main application window"""

  def createActionStub(self) -> None:
    """Omitting..."""
    pass

  def __init__(self, *args, **kwargs) -> None:
    """Initializes the main application window"""
    PubLayoutWindow.__init__(self, *args, **kwargs)

  def connectActions(self) -> None:
    """Initializes the user interface"""
    self.initButton.clicked.connect(self._initNote)
