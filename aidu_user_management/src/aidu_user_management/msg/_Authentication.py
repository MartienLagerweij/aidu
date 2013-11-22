"""autogenerated by genpy from aidu_user_management/Authentication.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import aidu_user_management.msg

class Authentication(genpy.Message):
  _md5sum = "04b6390d70e6dde142ab4c68a0774b73"
  _type = "aidu_user_management/Authentication"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool login
bool success
User user
================================================================================
MSG: aidu_user_management/User
string id
string first_name
string last_name
int32[] deliveries
"""
  __slots__ = ['login','success','user']
  _slot_types = ['bool','bool','aidu_user_management/User']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       login,success,user

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Authentication, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.login is None:
        self.login = False
      if self.success is None:
        self.success = False
      if self.user is None:
        self.user = aidu_user_management.msg.User()
    else:
      self.login = False
      self.success = False
      self.user = aidu_user_management.msg.User()

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
      buff.write(_struct_2B.pack(_x.login, _x.success))
      _x = self.user.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.user.first_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.user.last_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.user.deliveries)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.user.deliveries))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.user is None:
        self.user = aidu_user_management.msg.User()
      end = 0
      _x = self
      start = end
      end += 2
      (_x.login, _x.success,) = _struct_2B.unpack(str[start:end])
      self.login = bool(self.login)
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.id = str[start:end].decode('utf-8')
      else:
        self.user.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.first_name = str[start:end].decode('utf-8')
      else:
        self.user.first_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.last_name = str[start:end].decode('utf-8')
      else:
        self.user.last_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.user.deliveries = struct.unpack(pattern, str[start:end])
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
      buff.write(_struct_2B.pack(_x.login, _x.success))
      _x = self.user.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.user.first_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.user.last_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.user.deliveries)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.user.deliveries.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.user is None:
        self.user = aidu_user_management.msg.User()
      end = 0
      _x = self
      start = end
      end += 2
      (_x.login, _x.success,) = _struct_2B.unpack(str[start:end])
      self.login = bool(self.login)
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.id = str[start:end].decode('utf-8')
      else:
        self.user.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.first_name = str[start:end].decode('utf-8')
      else:
        self.user.first_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.user.last_name = str[start:end].decode('utf-8')
      else:
        self.user.last_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.user.deliveries = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2B = struct.Struct("<2B")