"""The 'layouts' provide for initialization of the widget at the moment
addWidget is invoked."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QVBoxLayout, QHBoxLayout
from ezside.widgets import BaseWidget


def addInit(cls: type) -> type:
  """Apply the extension to the class."""

  oldAddWidget = getattr(cls, 'addWidget', )

  def addWidget(this: cls, widget: BaseWidget, *args) -> None:
    """Add a widget to the layout."""
    if isinstance(widget, BaseWidget):
      widget.initUi()
    return oldAddWidget(this, widget)

  setattr(cls, 'addWidget', addWidget)
  return cls


@addInit
class Grid(QGridLayout):
  """GridLayout class provides a grid layout for the application."""

  def __init__(self, *args, **kwargs) -> None:
    QGridLayout.__init__(self, *args, **kwargs)


@addInit
class Vertical(QVBoxLayout):
  """VBoxLayout class provides a vertical layout for the application."""

  def __init__(self, *args, **kwargs) -> None:
    QVBoxLayout.__init__(self, *args, **kwargs)


@addInit
class Horizontal(QHBoxLayout):
  """HBoxLayout class provides a horizontal layout for the application."""

  def __init__(self, *args, **kwargs) -> None:
    QHBoxLayout.__init__(self, *args, **kwargs)