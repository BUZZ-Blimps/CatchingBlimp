# generated from rosidl_generator_py/resource/_idl.py.em
# with input from yolo_msgs:msg/BoundingBox.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BoundingBox(type):
    """Metaclass of message 'BoundingBox'."""

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
            module = import_type_support('yolo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'yolo_msgs.msg.BoundingBox')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bounding_box
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bounding_box
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bounding_box
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bounding_box
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bounding_box

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


class BoundingBox(metaclass=Metaclass_BoundingBox):
    """Message class 'BoundingBox'."""

    __slots__ = [
        '_header',
        '_x_center_balloon',
        '_y_center_balloon',
        '_width_balloon',
        '_height_balloon',
        '_x_center_y_goal',
        '_y_center_y_goal',
        '_width_y_goal',
        '_height_y_goal',
        '_x_center_o_goal',
        '_y_center_o_goal',
        '_width_o_goal',
        '_height_o_goal',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'x_center_balloon': 'int64',
        'y_center_balloon': 'int64',
        'width_balloon': 'int64',
        'height_balloon': 'int64',
        'x_center_y_goal': 'int64',
        'y_center_y_goal': 'int64',
        'width_y_goal': 'int64',
        'height_y_goal': 'int64',
        'x_center_o_goal': 'int64',
        'y_center_o_goal': 'int64',
        'width_o_goal': 'int64',
        'height_o_goal': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.x_center_balloon = kwargs.get('x_center_balloon', int())
        self.y_center_balloon = kwargs.get('y_center_balloon', int())
        self.width_balloon = kwargs.get('width_balloon', int())
        self.height_balloon = kwargs.get('height_balloon', int())
        self.x_center_y_goal = kwargs.get('x_center_y_goal', int())
        self.y_center_y_goal = kwargs.get('y_center_y_goal', int())
        self.width_y_goal = kwargs.get('width_y_goal', int())
        self.height_y_goal = kwargs.get('height_y_goal', int())
        self.x_center_o_goal = kwargs.get('x_center_o_goal', int())
        self.y_center_o_goal = kwargs.get('y_center_o_goal', int())
        self.width_o_goal = kwargs.get('width_o_goal', int())
        self.height_o_goal = kwargs.get('height_o_goal', int())

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
        if self.x_center_balloon != other.x_center_balloon:
            return False
        if self.y_center_balloon != other.y_center_balloon:
            return False
        if self.width_balloon != other.width_balloon:
            return False
        if self.height_balloon != other.height_balloon:
            return False
        if self.x_center_y_goal != other.x_center_y_goal:
            return False
        if self.y_center_y_goal != other.y_center_y_goal:
            return False
        if self.width_y_goal != other.width_y_goal:
            return False
        if self.height_y_goal != other.height_y_goal:
            return False
        if self.x_center_o_goal != other.x_center_o_goal:
            return False
        if self.y_center_o_goal != other.y_center_o_goal:
            return False
        if self.width_o_goal != other.width_o_goal:
            return False
        if self.height_o_goal != other.height_o_goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
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

    @property
    def x_center_balloon(self):
        """Message field 'x_center_balloon'."""
        return self._x_center_balloon

    @x_center_balloon.setter
    def x_center_balloon(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_center_balloon' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'x_center_balloon' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._x_center_balloon = value

    @property
    def y_center_balloon(self):
        """Message field 'y_center_balloon'."""
        return self._y_center_balloon

    @y_center_balloon.setter
    def y_center_balloon(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_center_balloon' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'y_center_balloon' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._y_center_balloon = value

    @property
    def width_balloon(self):
        """Message field 'width_balloon'."""
        return self._width_balloon

    @width_balloon.setter
    def width_balloon(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width_balloon' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'width_balloon' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._width_balloon = value

    @property
    def height_balloon(self):
        """Message field 'height_balloon'."""
        return self._height_balloon

    @height_balloon.setter
    def height_balloon(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height_balloon' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'height_balloon' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._height_balloon = value

    @property
    def x_center_y_goal(self):
        """Message field 'x_center_y_goal'."""
        return self._x_center_y_goal

    @x_center_y_goal.setter
    def x_center_y_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_center_y_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'x_center_y_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._x_center_y_goal = value

    @property
    def y_center_y_goal(self):
        """Message field 'y_center_y_goal'."""
        return self._y_center_y_goal

    @y_center_y_goal.setter
    def y_center_y_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_center_y_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'y_center_y_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._y_center_y_goal = value

    @property
    def width_y_goal(self):
        """Message field 'width_y_goal'."""
        return self._width_y_goal

    @width_y_goal.setter
    def width_y_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width_y_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'width_y_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._width_y_goal = value

    @property
    def height_y_goal(self):
        """Message field 'height_y_goal'."""
        return self._height_y_goal

    @height_y_goal.setter
    def height_y_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height_y_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'height_y_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._height_y_goal = value

    @property
    def x_center_o_goal(self):
        """Message field 'x_center_o_goal'."""
        return self._x_center_o_goal

    @x_center_o_goal.setter
    def x_center_o_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_center_o_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'x_center_o_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._x_center_o_goal = value

    @property
    def y_center_o_goal(self):
        """Message field 'y_center_o_goal'."""
        return self._y_center_o_goal

    @y_center_o_goal.setter
    def y_center_o_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_center_o_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'y_center_o_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._y_center_o_goal = value

    @property
    def width_o_goal(self):
        """Message field 'width_o_goal'."""
        return self._width_o_goal

    @width_o_goal.setter
    def width_o_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width_o_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'width_o_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._width_o_goal = value

    @property
    def height_o_goal(self):
        """Message field 'height_o_goal'."""
        return self._height_o_goal

    @height_o_goal.setter
    def height_o_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height_o_goal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'height_o_goal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._height_o_goal = value
