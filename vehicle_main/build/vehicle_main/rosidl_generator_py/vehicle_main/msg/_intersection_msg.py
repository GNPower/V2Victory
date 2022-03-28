# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vehicle_main:msg/IntersectionMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'directions'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IntersectionMsg(type):
    """Metaclass of message 'IntersectionMsg'."""

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
            module = import_type_support('vehicle_main')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vehicle_main.msg.IntersectionMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__intersection_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__intersection_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__intersection_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__intersection_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__intersection_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IntersectionMsg(metaclass=Metaclass_IntersectionMsg):
    """Message class 'IntersectionMsg'."""

    __slots__ = [
        '_position_x',
        '_position_y',
        '_num_directions',
        '_directions',
        '_intersection_state',
        '_intersection_next_state',
        '_intersection_switch_time',
    ]

    _fields_and_field_types = {
        'position_x': 'uint32',
        'position_y': 'uint32',
        'num_directions': 'uint8',
        'directions': 'uint32[32]',
        'intersection_state': 'uint8',
        'intersection_next_state': 'uint8',
        'intersection_switch_time': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 32),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.position_x = kwargs.get('position_x', int())
        self.position_y = kwargs.get('position_y', int())
        self.num_directions = kwargs.get('num_directions', int())
        if 'directions' not in kwargs:
            self.directions = numpy.zeros(32, dtype=numpy.uint32)
        else:
            self.directions = numpy.array(kwargs.get('directions'), dtype=numpy.uint32)
            assert self.directions.shape == (32, )
        self.intersection_state = kwargs.get('intersection_state', int())
        self.intersection_next_state = kwargs.get('intersection_next_state', int())
        self.intersection_switch_time = kwargs.get('intersection_switch_time', float())

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
        if self.position_x != other.position_x:
            return False
        if self.position_y != other.position_y:
            return False
        if self.num_directions != other.num_directions:
            return False
        if all(self.directions != other.directions):
            return False
        if self.intersection_state != other.intersection_state:
            return False
        if self.intersection_next_state != other.intersection_next_state:
            return False
        if self.intersection_switch_time != other.intersection_switch_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def position_x(self):
        """Message field 'position_x'."""
        return self._position_x

    @position_x.setter
    def position_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'position_x' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'position_x' field must be an unsigned integer in [0, 4294967295]"
        self._position_x = value

    @property
    def position_y(self):
        """Message field 'position_y'."""
        return self._position_y

    @position_y.setter
    def position_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'position_y' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'position_y' field must be an unsigned integer in [0, 4294967295]"
        self._position_y = value

    @property
    def num_directions(self):
        """Message field 'num_directions'."""
        return self._num_directions

    @num_directions.setter
    def num_directions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_directions' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'num_directions' field must be an unsigned integer in [0, 255]"
        self._num_directions = value

    @property
    def directions(self):
        """Message field 'directions'."""
        return self._directions

    @directions.setter
    def directions(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'directions' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 32, \
                "The 'directions' numpy.ndarray() must have a size of 32"
            self._directions = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 32 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'directions' field must be a set or sequence with length 32 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._directions = numpy.array(value, dtype=numpy.uint32)

    @property
    def intersection_state(self):
        """Message field 'intersection_state'."""
        return self._intersection_state

    @intersection_state.setter
    def intersection_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'intersection_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'intersection_state' field must be an unsigned integer in [0, 255]"
        self._intersection_state = value

    @property
    def intersection_next_state(self):
        """Message field 'intersection_next_state'."""
        return self._intersection_next_state

    @intersection_next_state.setter
    def intersection_next_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'intersection_next_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'intersection_next_state' field must be an unsigned integer in [0, 255]"
        self._intersection_next_state = value

    @property
    def intersection_switch_time(self):
        """Message field 'intersection_switch_time'."""
        return self._intersection_switch_time

    @intersection_switch_time.setter
    def intersection_switch_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'intersection_switch_time' field must be of type 'float'"
        self._intersection_switch_time = value
