"""BaseWidget provides the lowest level of abstraction for a QWidget. It is
controls only size limits."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QEnterEvent, QGuiApplication
from PySide6.QtWidgets import QWidget, QSizePolicy

from morevistutils.fields import Flag

MinExp = QSizePolicy.Policy.MinimumExpanding
Max = QSizePolicy.Policy.Maximum


class BaseWidget(QWidget):
  """BaseWidget provides the lowest level of abstraction for a QWidget. It is
  controls only size limits."""

  __under_mouse__ = None
  __button_down__ = None
  __enabled_state__ = None
  __activated_state__ = None

  underMouse = Flag(False)
  buttonDown = Flag(False)
  enabled = Flag(False)
  activated = Flag(False)

  @underMouse.GET
  def getUnderMouse(self, ) -> bool:
    """Getter-function for the underMouse flag."""
    return self.__under_mouse__

  @buttonDown.GET
  def getButtonDown(self, ) -> bool:
    """Getter-function for the buttonDown flag."""
    if self.underMouse:
      return True if QGuiApplication.mouseButtons().value else False
    return False

  @enabled.GET
  def getEnabled(self, ) -> bool:
    """Getter-function for the enabled flag."""
    return self.__enabled_state__

  @enabled.SET
  def _wrapSetEnabled(self, value: bool) -> None:
    """Setter-function for the enabled flag. """
    self.enable() if value else self.disable()

  @activated.GET
  def getActivated(self, ) -> bool:
    """Getter-function for the activated flag."""
    return self.__activated_state__

  @activated.SET
  def _wrapSetActivated(self, value: bool) -> None:
    """Setter-function for the activated flag."""
    self.activate() if value else self.deactivate()

  def enable(self, ) -> None:
    """Setter-function for the enabled flag. Subclasses must implement
    this method for the flag to be available. """

  def disable(self, ) -> None:
    """Setter-function for the enabled flag. Subclasses must implement
    this method for the flag to be available. """

  def activate(self, ) -> None:
    """Setter-function for the activated flag. Subclasses must implement
    this method for the flag to be available. """

  def deactivate(self, ) -> None:
    """Setter-function for the activated flag. Subclasses must implement
    this method for the flag to be available. """

  def __init__(self, *args, **kwargs) -> None:
    for arg in args:
      if isinstance(arg, QWidget):
        QWidget.__init__(self, arg)
        break
    else:
      QWidget.__init__(self, )

  def enterEvent(self, event: QEnterEvent) -> None:
    """Event for entering the widget."""
    QWidget.enterEvent(self, event)
    self.__under_mouse__ = True

  def leaveEvent(self, event: QEnterEvent) -> None:
    """Event for leaving the widget."""
    QWidget.leaveEvent(self, event)
    self.__under_mouse__ = False
