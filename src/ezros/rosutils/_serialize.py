"""The serialize function receives a message type and a message insatnce
and translates to a timestamped value."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from msgs.msg import Float32Stamped


def serialize(messageType: type, message: Any) -> Any:
  """Serialize the message."""
  if messageType is Float32Stamped:
    return message.header.stamp.to_sec() + message.data * 1j
  raise NotImplementedError(f"Message type {messageType} is not supported.")
