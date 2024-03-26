"""The MainWindow class organizes the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from warnings import warn

from PySide6.QtWidgets import QMainWindow
from icecream import ic
from rospy import Subscriber, init_node
from vistutils.waitaminute import typeMsg

from ezros.app import (LayoutWindow)
from ezros.rosutils import resolveTopicType

ic.configureOutput(includeContext=True)


class MainWindow(LayoutWindow):
  """The MainWindow class organizes the main application window."""

  __noise_subscriber__ = None

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('EZROS')
    self.resize(800, 600)

  def createSubscriber(self) -> None:
    """Create a subscriber."""
    if self.__noise_subscriber__ is None:
      name = 'yolo'
      type_ = resolveTopicType(name)
      callback = self.dynChart.append
      init_node('lmao', anonymous=True)
      self.__noise_subscriber__ = Subscriber('yolo', type_, callback)
    w = """Subscriber already exists."""
    warn(w)

  def getSubscriber(self, **kwargs) -> Subscriber:
    """Get the subscriber."""
    if self.__noise_subscriber__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createSubscriber()
      return self.getSubscriber(_recursion=True)
    if isinstance(self.__noise_subscriber__, Subscriber):
      return self.__noise_subscriber__
    name = 'self.__noise_subscriber__'
    value = self.__noise_subscriber__
    expected = Subscriber
    e = typeMsg(name, value, expected)
    raise TypeError(e)

  def connectActions(self) -> None:
    """Connect actions."""

  def show(self) -> None:
    """Show"""
    self.initMenus()
    self.initUi()
    self.connectActions()
    QMainWindow.show(self)
