"""ResizeFilter provides an event filter class to detect resize events and
defer the repaint of all widgets within a main window until resizing is
completed."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QEvent, QObject, QTimer
from PySide6.QtWidgets import QWidget


class ResizeFilter(QObject):
  """
  An event filter class to detect resize events and defer the
  repaint of all widgets within a main window until resizing is completed.
  """

  def __init__(self, parent: QWidget, delay: int = 100):
    super().__init__(parent)
    self.delay = delay
    self.resizeTimer = QTimer(self)
    self.resizeTimer.setSingleShot(True)
    self.resizeTimer.timeout.connect(self.onResizeTimeout)
    self.installOnChildren(parent)
    self.parent = parent

  def installOnChildren(self, parent: QWidget):
    """
    Recursively install this event filter on the parent and all its
    child widgets.
    """
    parent.installEventFilter(self)
    for child in parent.findChildren(QWidget):
      child.installEventFilter(self)

  def eventFilter(self, obj: QObject, event: QEvent) -> bool:
    """Implementation of parent method"""
    if event.type() == QEvent.Type.Resize:
      self.resizeTimer.start(self.delay)
      return False  # Return False to continue normal event processing
    if event.type() == QEvent.Type.Paint and self.resizeTimer.isActive():
      return True
    return super().eventFilter(obj, event)

  def onResizeTimeout(self) -> None:
    """The resize timeout triggers a repaint"""
    self.parent.update()
