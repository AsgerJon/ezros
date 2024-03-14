"""This packages contains the classes for the GUI of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from ._client_info import ClientInfo, ClientInfoField
from ._ping_indicator import PingIndicator, PingIndicatorField
from ._op_state import OpState, OpStateField, OpSelect, OpField
from ._connection_status import ConnectionStatus, ConnectionStatusField
from ._dyn_plot import DynPlot, DynPlotField
from ._tab_field import TabField, TabWidget
from ._data_field import DataField, DataSeries
from ._data_axis import Axis, AxisField
