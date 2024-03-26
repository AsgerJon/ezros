"""StrInput widget for inputting text. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal
from attribox import AttriBox, this, scope
from ezside.widgets import BaseWidget
from vistutils.parse import maybe
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from ezros.widgets import Horizontal, TightLabel, LineEdit, PushButton


class StrInput(BaseWidget):
  """StrInput widget for inputting text. """

  __fallback_label__ = 'Input'
  __fallback_button__ = 'Submit'
  __fallback_current__ = 'Current'
  __fallback_placeholder__ = 'Enter text here'

  baseLayout = AttriBox[Horizontal]()
  label = AttriBox[TightLabel](scope)
  lineEdit = AttriBox[LineEdit](scope)
  button = AttriBox[PushButton](scope)
  current = AttriBox[TightLabel](scope)

  newValue = Signal(str, str)
  valueChanged = Signal()

  def __init__(self, *args, **kwargs) -> None:
    """Initialize the widget."""
    BaseWidget.__init__(self, *args, **kwargs)
    labelArg, buttonArg, currentArg, placeholderArg = None, None, None, None
    for arg in args:
      if isinstance(arg, BaseWidget):
        self.__label_title__ = getattr(arg, '__label_title__', )
        self.__button_text__ = getattr(arg, '__button_text__', )
        self.__current_text__ = getattr(arg, '__current_text__', )
        self.__placeholder_text__ = getattr(arg, '__placeholder_text__', )
        break
    else:
      labelKeys = stringList("""label, title, text, name, header""")
      buttonKeys = stringList("""button, apply, submit""")
      currentKeys = stringList("""current, existing""")
      phKeys = stringList("""placeholder, hint, example""")
      Keys = [labelKeys, buttonKeys, currentKeys, phKeys]
      data = dict(label=str, button=str, current=str, placeholder=str)
      for (keys, (name, type_)) in zip(Keys, data.items()):
        for key in keys:
          if key in kwargs:
            val = kwargs.get(key)
            if isinstance(val, type_):
              data[name] = val
              break
            e = typeMsg(name, val, type_)
            raise TypeError(e)
        else:
          for arg in args:
            if isinstance(arg, type_):
              data[name] = arg
              break
          else:
            data[name] = getattr(self.__class__, '__fallback_%s__' % name)
      self.__label_title__ = data['label']
      self.__button_text__ = data['button']
      self.__current_text__ = data['current']
      self.__placeholder_text__ = data['placeholder']

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.baseLayout.addWidget(self.label)
    self.baseLayout.addWidget(self.lineEdit)
    self.baseLayout.addWidget(self.button)
    self.baseLayout.addWidget(self.current)
    self.setLayout(self.baseLayout)

  def initActions(self) -> None:
    """Initialize the actions."""
    self.button.clicked.connect(self.handleButton)
    self.lineEdit.returnPressed.connect(self.handleButton)

  def handleButton(self, ) -> None:
    """Handle the button."""
    self.valueChanged.emit()
    oldValue = self.current.text()
    newValue = self.lineEdit.text()
    self.newValue(newValue, oldValue)
    self.valueChanged.emit()
    self.current.setText(newValue)
    self.lineEdit.clear()
