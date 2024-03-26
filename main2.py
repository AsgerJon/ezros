"""lmao"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
from PySide6.QtCore import QSize
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow, QApplication, QMenuBar
from icecream import ic

ic.configureOutput(includeContext=True)


class MainWindow(QMainWindow):
  """The MainWindow class organizes the main application window."""

  def __init__(self, *args, **kwargs) -> None:
    QMainWindow.__init__(self, *args, **kwargs)
    self.setWindowTitle('LOL')
    self.setMinimumSize(QSize(320, 240))
    self.createMenuBar()

  def createMenuBar(self) -> None:
    """Creates and assigns a custom menu bar."""
    menuBar: QMenuBar = QMenuBar(self)
    self.setMenuBar(menuBar)

    # File menu
    fileMenu = menuBar.addMenu("&File")

    # Exit action
    exitAction: QAction = QAction("&Exit", self)
    exitAction.triggered.connect(self.close)
    fileMenu.addAction(exitAction)

    # Help menu
    helpMenu = menuBar.addMenu("&Help")

    # About action
    aboutAction: QAction = QAction("&About", self)
    aboutAction.triggered.connect(QApplication.aboutQt)
    helpMenu.addAction(aboutAction)


if __name__ == '__main__':
  app = QApplication(sys.argv)
  main = MainWindow()
  main.show()
  app.exec()
