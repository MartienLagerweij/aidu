"""autogenerated by genpy from aidu_mobile_base/BaseState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import aidu_mobile_base.msg

class BaseState(genpy.Message):
  _md5sum = "ba1e4aac8a0200f8ba0b1bbcc7ebec76"
  _type = "aidu_mobile_base/BaseState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# States of the base components 
State left
State right
State angle
================================================================================
MSG: aidu_mobile_base/State
# The current state of a base component
float32 position
float32 speed
"""
  __slots__ = ['left','right','angle']
  _slot_types = ['aidu_mobile_base/State','aidu_mobile_base/State','aidu_mobile_base/State']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       left,right,angle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BaseState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.left is None:
        self.left = aidu_mobile_base.msg.State()
      if self.right is None:
        self.right = aidu_mobile_base.msg.State()
      if self.angle is None:
        self.angle = aidu_mobile_base.msg.State()
    else:
      self.left = aidu_mobile_base.msg.State()
      self.right = aidu_mobile_base.msg.State()
      self.angle = aidu_mobile_base.msg.State()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_6f.pack(_x.left.position, _x.left.speed, _x.right.position, _x.right.speed, _x.angle.position, _x.angle.speed))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.left is None:
        self.left = aidu_mobile_base.msg.State()
      if self.right is None:
        self.right = aidu_mobile_base.msg.State()
      if self.angle is None:
        self.angle = aidu_mobile_base.msg.State()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.left.position, _x.left.speed, _x.right.position, _x.right.speed, _x.angle.position, _x.angle.speed,) = _struct_6f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_6f.pack(_x.left.position, _x.left.speed, _x.right.position, _x.right.speed, _x.angle.position, _x.angle.speed))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.left is None:
        self.left = aidu_mobile_base.msg.State()
      if self.right is None:
        self.right = aidu_mobile_base.msg.State()
      if self.angle is None:
        self.angle = aidu_mobile_base.msg.State()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.left.position, _x.left.speed, _x.right.position, _x.right.speed, _x.angle.position, _x.angle.speed,) = _struct_6f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6f = struct.Struct("<6f")