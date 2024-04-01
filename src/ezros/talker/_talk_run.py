"""TalkRun provides the script running the talker application."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys

from ezros.talker import TalkApp, TalkWindow


def main() -> None:
  """Run the talker application."""
  app = TalkApp(sys.argv)
  window = TalkWindow()
  window.show()
  sys.exit(app.exec())


if __name__ == '__main__':
  main()
