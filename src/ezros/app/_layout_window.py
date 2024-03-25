"""LayoutWindow organizes the layouts used in the application"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any
from warnings import warn

from PySide6.QtWidgets import QVBoxLayout
from attribox import AttriBox
from ezside import BaseWindow
from ezside.widgets import BaseWidget, TextLabel, DataView
from icecream import ic
from rospy import Subscriber, init_node
from vistutils.text import monoSpace
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType
from ezros.widgets import TabWidget, DynChart

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  baseWidget = AttriBox[BaseWidget]()
  baseLayout = AttriBox[QVBoxLayout]()
  dynChart = AttriBox[DynChart]()

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    name = 'yolo'
    type_ = resolveTopicType(name)
    callback = self.dynChart.append
    init_node('lmao', anonymous=True)
    self.sub = Subscriber('yolo', resolveTopicType('yolo'), callback)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.dynChart.initUi()
    self.baseLayout.addWidget(self.dynChart)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
    BaseWindow.initUi(self)

  def show(self) -> None:
    """Show the window."""
    self.initUi()
    BaseWindow.show(self)

  def preAppend(self, data: Any) -> None:
    """Extracts the floating point value from the data instance before
    passing the data on to the dataView"""
    dataStr = str(data)
    typeName = data.__class__.__qualname__
    try:
      value = getattr(data, 'data', )
    except AttributeError as attributeError:
      e = """Unable to extract floating point value from given data 
      structure: '%s' of type: '%s'!""" % (dataStr, typeName)
      raise ValueError(monoSpace(e)) from attributeError
    if value is None:
      w = """Found value 'None' when attempting to extract floating point 
      value from data: '%s' of type: '%s'!""" % (dataStr, typeName)
      warn(monoSpace(w))
    if isinstance(value, (int, float)):
      return self.dynChart.append(float(value))
    e = typeMsg('value', value, float)
    raise TypeError(e)
