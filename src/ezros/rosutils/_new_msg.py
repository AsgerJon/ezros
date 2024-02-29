"""ROS msg generator. Why?"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.metas import AbstractNamespace
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg

from morevistutils import Field


class RosField(Field):
  """A ROS field is a field in a ROS message"""

  __type_keys__ = None
  __field_py_type__ = None
  __field_ros_type__ = None
  __field_ros_array_type__ = None

  @classmethod
  def _createTypeKeys(cls) -> None:
    """Creator-function for type keys"""
    cls.__type_keys__ = stringList("""type, msgType, msgTypeArray, 
    pythonType, stdMsgType, stdMsgArrayType, stdMsgArray, 
    stdMsgArrayMsgType,""")

  @classmethod
  def _getTypeKeys(cls, **kwargs) -> list[str]:
    """Getter-function for type keys"""
    if cls.__type_mapping__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      cls._createTypeMapping()
      return cls._getTypeKeys(_recursion=True)
    if isinstance(cls.__type_keys__, list):
      return cls.__type_keys__
    e = typeMsg('type_keys', cls.__type_keys__, list)
    raise TypeError(e)

  @staticmethod
  def _resolveType(key: Any) -> dict[str, type | None]:
    """Resolves the type name"""
    if isinstance(key, str):
      if key in RosField._getTypeKeys():
        val = RosField._getTypeMapping().get(key)
        if isinstance(val, dict):
          if 'python' in val and 'std_msg' in val:
            return dict(pythonType=val['python'],
                        stdMsgType=val['std_msg'],
                        stdMsgArrayType=val.get('std_msg_array', None))

  def __init__(self, *args, **kwargs) -> None:
    Field.__init__(self, *args, **kwargs)
    for key in self._getTypeKeys():
      if key in kwargs:
        pass


class ROSNameSpace(AbstractNamespace):
  """ROSMsg is a factory for generating ROS message classes"""

  __msg_type_name__ = None
  __msg_type_fields__ = None
