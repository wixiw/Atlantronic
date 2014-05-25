"""autogenerated by genmsg_py from LocalizationState.msg. Do not edit."""
import roslib.message
import struct


class LocalizationState(roslib.message.Message):
  _md5sum = "44a86e9e98f3fe7590353783d087e953"
  _type = "arp_core/LocalizationState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int64 state # STOPPED = 0, RUNNING = 1
int64 mode # ODO_ONLY = 0, SMOOTH = 1, FUSION = 2
int64 quality # LOST = 0, BAD = 1, GOOD = 2
int64 visibility #NONE = 0, OCCULTED = 1, SEGMENT = 2, TRIANGLE = 3


"""
  __slots__ = ['state','mode','quality','visibility']
  _slot_types = ['int64','int64','int64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       state,mode,quality,visibility
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(LocalizationState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.state is None:
        self.state = 0
      if self.mode is None:
        self.mode = 0
      if self.quality is None:
        self.quality = 0
      if self.visibility is None:
        self.visibility = 0
    else:
      self.state = 0
      self.mode = 0
      self.quality = 0
      self.visibility = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_4q.pack(_x.state, _x.mode, _x.quality, _x.visibility))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.state, _x.mode, _x.quality, _x.visibility,) = _struct_4q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_4q.pack(_x.state, _x.mode, _x.quality, _x.visibility))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.state, _x.mode, _x.quality, _x.visibility,) = _struct_4q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_4q = struct.Struct("<4q")
