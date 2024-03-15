"""This packages contains the classes for the GUI of the application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._dyn_plot import DynPlot
from ._tab_field import TabWidget
from ._client_info import ClientInfo, ClientInfoField
from ._ping_indicator import PingIndicator, PingIndicatorField
from ._op_state import OpState, OpStateField, OpSelect, OpField
from ._connection_status import ConnectionStatus, ConnectionStatusField
from ._data_field import DataField, DataSeries
from ._axis_field import Axis, AxisField
from ._chart_field import ChartField, DataChart
from ._data_view import DataView, ViewField
from ._data_widget import PlotWidget, HookedView, PlotField
from ._data_chart import DataChart
