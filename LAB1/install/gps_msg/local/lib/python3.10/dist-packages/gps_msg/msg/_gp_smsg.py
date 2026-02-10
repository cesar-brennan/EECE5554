# generated from rosidl_generator_py/resource/_idl.py.em
# with input from gps_msg:msg/GPSmsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GPSmsg(type):
    """Metaclass of message 'GPSmsg'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('gps_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'gps_msg.msg.GPSmsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gp_smsg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gp_smsg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gp_smsg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gp_smsg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gp_smsg

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GPSmsg(metaclass=Metaclass_GPSmsg):
    """Message class 'GPSmsg'."""

    __slots__ = [
        '_header',
        '_latitude',
        '_longitude',
        '_altitude',
        '_hdop',
        '_utm_easting',
        '_utm_northing',
        '_utc',
        '_zone',
        '_letter',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'double',
        'hdop': 'double',
        'utm_easting': 'double',
        'utm_northing': 'double',
        'utc': 'double',
        'zone': 'string',
        'letter': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.hdop = kwargs.get('hdop', float())
        self.utm_easting = kwargs.get('utm_easting', float())
        self.utm_northing = kwargs.get('utm_northing', float())
        self.utc = kwargs.get('utc', float())
        self.zone = kwargs.get('zone', str())
        self.letter = kwargs.get('letter', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.hdop != other.hdop:
            return False
        if self.utm_easting != other.utm_easting:
            return False
        if self.utm_northing != other.utm_northing:
            return False
        if self.utc != other.utc:
            return False
        if self.zone != other.zone:
            return False
        if self.letter != other.letter:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._altitude = value

    @builtins.property
    def hdop(self):
        """Message field 'hdop'."""
        return self._hdop

    @hdop.setter
    def hdop(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'hdop' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'hdop' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._hdop = value

    @builtins.property
    def utm_easting(self):
        """Message field 'utm_easting'."""
        return self._utm_easting

    @utm_easting.setter
    def utm_easting(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'utm_easting' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'utm_easting' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._utm_easting = value

    @builtins.property
    def utm_northing(self):
        """Message field 'utm_northing'."""
        return self._utm_northing

    @utm_northing.setter
    def utm_northing(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'utm_northing' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'utm_northing' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._utm_northing = value

    @builtins.property
    def utc(self):
        """Message field 'utc'."""
        return self._utc

    @utc.setter
    def utc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'utc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'utc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._utc = value

    @builtins.property
    def zone(self):
        """Message field 'zone'."""
        return self._zone

    @zone.setter
    def zone(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'zone' field must be of type 'str'"
        self._zone = value

    @builtins.property
    def letter(self):
        """Message field 'letter'."""
        return self._letter

    @letter.setter
    def letter(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'letter' field must be of type 'str'"
        self._letter = value
