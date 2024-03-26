"""The addInit function decorates the layout classes such that the widget
is initialized at the moment addWidget is invoked.  """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

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
