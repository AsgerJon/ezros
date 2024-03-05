"""RosType provides conversion between ROS and Python types."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from std_msgs.msg import UInt32, Byte, Bool, String, Float64, Int32, \
  ByteMultiArray
from std_msgs.msg import Int8, Int16, Int64, UInt8, UInt16, UInt64
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray, Int16MultiArray
from std_msgs.msg import Int32MultiArray, Int64MultiArray
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray
from std_msgs.msg import UInt32MultiArray, UInt64MultiArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from vistutils.metas import AbstractNamespace
from vistutils.text import stringList
from vistutils.waitaminute import typeMsg


class RosType(AbstractNamespace):
  """RosType provides conversion between ROS and Python types."""

  @staticmethod
  def _isDunder(name: str) -> bool:
    """Checks if the name is a dunder name"""
    if name.startswith('__'):
      if name.endswith('__'):
        return True
    return False

  __type_mapping__ = None

  @classmethod
  def _createTypeMapping(cls) -> None:
    """Creator-function for the type mapping"""
    base = {
      'int'  : {
        'pyType'  : int,
        'rosTypes': (Int8, Int16, Int32, Int64),
        'rosPref' : Int64,
        'rosArray': (Int8MultiArray,
                     Int16MultiArray,
                     Int32MultiArray,
                     Int64MultiArray,)
      },
      'float': {
        'pyType'  : float,
        'rosType' : (Float32, Float64,),
        'rosPref' : Int64,
        'rosArray': (Float32MultiArray,
                     Float64MultiArray,),
      },
      'str'  : {
        'pyType'  : str,
        'rosType' : String,
        'rosPref' : String,
        'rosArray': None
      },
      'bool' : {
        'pyType'  : bool,
        'rosType' : Bool,
        'rosPref' : Bool,
        'rosArray': None
      },
      'byte' : {
        'pyType'  : bytes,
        'rosType' : Byte,
        'rosPref' : Byte,
        'rosArray': ByteMultiArray,
      },
      'uint' : {
        'pyType'  : int,
        'rosType' : (UInt8, UInt16, UInt32, UInt64),
        'rosPref' : Int64,
        'rosArray': (UInt8MultiArray,
                     UInt16MultiArray,
                     UInt32MultiArray,
                     UInt64MultiArray,)
      }
    }
    cls.__type_mapping__ = {}
    for (pyName, val) in base.items():
      rosTypes = val['rosTypes']
      rosArrays = val['rosArray']
      if rosArrays is None:
        rosArrays = ()
      pyType = val['pyType']
      typeKeys = [pyType, *rosTypes, *rosArrays]
      strKeys = [t.__name__ for t in typeKeys]
      for key in [*strKeys, *typeKeys]:
        cls.__type_mapping__[key] = val

  @classmethod
  def _getTypeMapping(cls, **kwargs) -> dict[str, dict[str, str]]:
    """Getter-function for the type mapping"""
    if cls.__type_mapping__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      cls._createTypeMapping()
      return cls._getTypeMapping(_recursion=True)
    if isinstance(cls.__type_mapping__, dict):
      return cls.__type_mapping__
    e = typeMsg('type_mapping', cls.__type_mapping__, dict)
    raise TypeError(e)

  def __init__(self, *args, **kwargs) -> None:
    AbstractNamespace.__init__(self, *args, **kwargs)

  def __getitem__(self, key: str) -> Any:
    try:
      return AbstractNamespace.__getitem__(self, key)
    except KeyError as keyError:
      if key in str(keyError):
        if key in self._getTypeMapping():
          return self._getTypeMapping()[key]
        raise keyError

  def __setitem__(self, key: str, val: Any) -> None:
    if key in self._getTypeMapping():
      e = """Found name: '%s' which is a reserved name!""" % key
      raise NameError(e)
    AbstractNamespace.__setitem__(self, key, val)

  def __delitem__(self, key: str) -> None:
    if key in self._getTypeMapping():
      e = """Found name: '%s' which is a reserved name!""" % key
      raise NameError(e)
    AbstractNamespace.__delitem__(self, key)

  def compile(self) -> dict:
    """Compiles the namespace object actually passed on to the class
    creation."""
    namespace = {}
    for (key, val) in self.getAnnotations().items():
      if val in self._getTypeMapping() and not self._isDunder(key):
        fieldName = key
        namespace[fieldName] = dict(fieldType=val.get('pyType', ),
                                    rosType=val.get('rosPref', ),
                                    rosAlts=val.get('rosTypes', ),
                                    rosArray=val.get('rosArray', ), )
      elif callable(val) or self._isDunder(key):
        namespace[key] = val
      else:
        e = """Classes derived from RosMeta are expected to have 
        attributes that are either: types to become fields in the 
        resulting message type or callables. Only dunder names are exempt 
        from this requirement.
        
        Class '%s' tried to set an unsupported attribute: '%s' at name: '%s'!
        """ % (self.__class_name__, val, key)
        raise AttributeError(e)
    return namespace
