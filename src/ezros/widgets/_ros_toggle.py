"""RosToggle provides toggle control and indication of a particular dual
state topic."""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QPushButton
from attribox import AttriBox
from ezside.core import Expand, Tight
from ezside.widgets import BaseWidget, \
  Label, \
  Grid, \
  HorizontalSpacer, \
  VerticalSpacer
from msgs.msg import AuxCommand
from rospy import Duration, Time
from vistutils.waitaminute import typeMsg

from ezros.rosutils import resolveTopicType, BoolPubRos
from ezros.utils import EmptyField
from ezros.widgets import Button


class RosToggle(BaseWidget):
  """RosToggle provides toggle control and indication of a particular dual
  state topic. """

  __inner_state__ = None
  __activator_button__ = None
  __deactivator_button__ = None
  __fallback_state__ = False
  __topic_name__ = None
  __pub_thread__ = None

  state = EmptyField()
  topicName = EmptyField()
  pubThread = EmptyField()
  activator = EmptyField()
  deactivator = EmptyField()

  baseLayout = AttriBox[Grid]()
  hSpacer = AttriBox[HorizontalSpacer]()
  v1Spacer = AttriBox[VerticalSpacer]()
  v2Spacer = AttriBox[VerticalSpacer]()
  stateLabel = AttriBox[Label]()

  stateChanged = Signal(bool)
  activated = Signal()
  deactivated = Signal()
  activateClicked = Signal()
  deactivateClicked = Signal()

  @activator.GET
  def _getActivator(self) -> QPushButton:
    """Getter-function for the activator."""
    if self.__activator_button__ is None:
      self.__activator_button__ = QPushButton()
      self.__activator_button__.setText('- ON -')
      self.__activator_button__.setEnabled(True)
      self.__activator_button__.clicked.connect(self.activate)
    return self.__activator_button__

  @deactivator.GET
  def _getDeactivator(self) -> QPushButton:
    """Getter-function for the deactivator."""
    if self.__deactivator_button__ is None:
      self.__deactivator_button__ = QPushButton()
      self.__deactivator_button__.setText('- OFF -')
      self.__deactivator_button__.setEnabled(False)
      self.__deactivator_button__.clicked.connect(self.deactivate)
    return self.__deactivator_button__

  @state.GET
  def _getState(self, **kwargs) -> bool:
    """Getter-function for the state."""
    if self.__inner_state__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.__inner_state__ = self.__fallback_state__
      return self._getState(_recursion=True)
    return True if self.__inner_state__ else False

  @state.SET
  def _setState(self, state: Any) -> None:
    """Setter-function for the state"""
    self.__inner_state__ = True if state else False

  def __bool__(self, ) -> bool:
    """Return the boolean value of the toggle."""
    return self._getState()

  @topicName.GET
  def _getTopicName(self) -> str:
    """Getter-function for the topic name."""
    return self.__topic_name__

  @topicName.SET
  def _setTopicName(self, topicName: str) -> None:
    """Setter-function for the topic name."""
    self.__topic_name__ = self._validateTopicName(topicName)
    self.updateLabel()

  @staticmethod
  def _validateTopicName(topicName: str) -> str:
    """Validate the topic name."""
    topicType = resolveTopicType(topicName)
    if topicType is not AuxCommand:
      e = 'The topic type must be AuxCommand'
      raise TypeError(e)
    return topicName

  @Slot()
  def activate(self) -> None:
    """Activate the state."""
    if not self:
      self.state = True
      self.activated.emit()
      self.stateChanged.emit(True)
      self.activator.setEnabled(False)
      self.deactivator.setEnabled(True)

  @Slot()
  def deactivate(self) -> None:
    """Deactivate the state."""
    if self:
      self.state = False
      self.deactivated.emit()
      self.stateChanged.emit(False)
      self.activator.setEnabled(True)
      self.deactivator.setEnabled(False)

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.activator.setText('- ON -')
    self.deactivator.setText('- OFF -')
    self.baseLayout.addWidget(self.stateLabel, 0, 0, 1, 3)
    self.baseLayout.addWidget(self.activator, 1, 0)
    self.baseLayout.addWidget(self.hSpacer, 1, 1, 1, 1)
    self.baseLayout.addWidget(self.deactivator, 1, 2)
    # self.baseLayout.addWidget(self.v1Spacer, 2, 0, 1, 1)
    # self.baseLayout.addWidget(self.v2Spacer, 2, 2, 1, 1)
    self.setLayout(self.baseLayout)
    self.activator.setEnabled(False if self else True)
    self.deactivator.setEnabled(True if self else False)
    self.setSizePolicy(Tight, Expand)

  def connectActions(self) -> None:
    """Connect the actions."""
    self.activator.clicked.connect(self.activate)
    self.activator.clicked.connect(self.activateClicked)
    self.deactivator.clicked.connect(self.deactivate)
    self.deactivator.clicked.connect(self.deactivateClicked)
    # self.stateChanged.connect(self.stateChanged.emit)
    self.stateChanged.connect(self.updateLabel)
    if isinstance(self.pubThread, BoolPubRos):
      self.pubThread.stateChanged.connect(self._setState)
      self.pubThread.activated.connect(self.activated)
      self.pubThread.deactivated.connect(self.deactivated)
    else:
      e = typeMsg('pubThread', self.pubThread, BoolPubRos)
      raise TypeError(e)

  def updateLabel(self, ) -> None:
    """Update the label."""
    updatedText = '%s: %s' % (self.topicName, 'ON' if self else 'OFF')
    self.stateLabel.text = updatedText

  def _createPubThread(self) -> None:
    """Create the publisher thread."""
    if self.__pub_thread__ is None:
      if isinstance(self.topicName, str):
        self.__pub_thread__ = BoolPubRos(self.topicName)
      else:
        e = typeMsg('topicName', self.topicName, str)
        raise TypeError(e)
    else:
      e = """The publisher thread has already been created!"""
      raise RuntimeError(e)

  @pubThread.GET
  def _getPubThread(self, **kwargs) -> BoolPubRos:
    """Getter-function for the publisher thread."""
    if self.__pub_thread__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPubThread()
      return self._getPubThread(_recursion=True)
    if isinstance(self.__pub_thread__, BoolPubRos):
      return self.__pub_thread__
    e = typeMsg('pub_thread', self.__pub_thread__, BoolPubRos)
    raise TypeError(e)

  def __init__(self, topicName: str) -> None:
    """Initialize the RosToggle instance."""
    BaseWidget.__init__(self)
    self.topicName = topicName
