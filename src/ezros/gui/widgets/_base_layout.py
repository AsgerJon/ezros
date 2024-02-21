"""BaseLayout subclasses the QGridLayout class. The subclassing ensures
compatibility with the Field class, but adds nothing more to the class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QWidget


class BaseLayout(QGridLayout):
  """BaseLayout subclasses the QGridLayout class. The subclassing ensures
  compatibility with the Field class, but adds nothing more to the class. """

  def __init__(self, *args, **kwargs) -> None:
    for arg in args:
      if isinstance(arg, QWidget):
        QGridLayout.__init__(self, arg)
        break
    else:
      QGridLayout.__init__(self, )
