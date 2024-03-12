"""ConnectionStatus shows the present connection status of the robot. In
particular the ping time. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistside.widgets import BaseWidget, BaseLayoutField, LabelField

from ezros.gui import PingIndicatorField


class ConnectionStatus(BaseWidget):
  """ConnectionStatus shows the present connection status of the robot. In
  particular the ping time. """

  baseLayout = BaseLayoutField(layout='vertical')
  headerLabel = LabelField(text='Connection Status')
  pingIndicator = PingIndicatorField()
  
