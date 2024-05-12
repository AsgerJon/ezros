"""LayoutWindow organizes the layouts used in the application"""
#  GPL-3.0 license
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QVBoxLayout, QLineEdit, QHBoxLayout
from ezside.app import BaseWindow
from ezside.core import AlignTop, AlignLeft, Expand, Tight
from ezside.widgets import BaseWidget, LiveChart, PushButton, Label
from icecream import ic

from ezros.widgets import RosToggle, TopicInfo

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """The LayoutWindow class provides a base class for all windows in the
  application. """

  debug1: QAction
  debug2: QAction
  debug3: QAction
  debug4: QAction
  debug5: QAction
  debug6: QAction
  debug7: QAction
  debug8: QAction
  debug9: QAction
  baseLayout: QVBoxLayout
  baseWidget: BaseWidget
  topicInfo: TopicInfo
  lineEdit: QLineEdit
  submitButton: PushButton
  codeLayout: QHBoxLayout
  codeWidget: BaseWidget
  vSpacer: Label

  @abstractmethod
  def initSignalSlot(self) -> None:
    """Initialize the signal-slot connections."""
    pass

  def initUi(self) -> None:
    """Initialize the user interface."""
    self.setMinimumSize(480, 320)
    self.baseLayout = QVBoxLayout()
    self.baseLayout.setAlignment(AlignTop | AlignLeft)
    self.baseLayout.setContentsMargins(0, 0, 0, 0, )
    self.baseLayout.setSpacing(0)
    self.baseWidget = BaseWidget()
    self.topicInfo = TopicInfo()
    self.topicInfo.initUi()
    self.baseLayout.addWidget(self.topicInfo)
    self.lineEdit = QLineEdit()
    self.lineEdit.setPlaceholderText('Enter code here...')
    self.submitButton = PushButton('Submit')
    self.submitButton.initUi()
    self.codeWidget = BaseWidget()
    self.codeLayout = QHBoxLayout()
    self.codeLayout.setContentsMargins(0, 0, 0, 0, )
    self.codeLayout.setSpacing(0)
    self.codeLayout.addWidget(self.lineEdit)
    self.codeLayout.addWidget(self.submitButton)
    self.codeWidget.setLayout(self.codeLayout)
    self.baseLayout.addWidget(self.codeWidget)
    self.vSpacer = Label('LMAO')
    self.vSpacer.setSizePolicy(Tight, Expand)
    self.baseLayout.addWidget(self.vSpacer)

    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
