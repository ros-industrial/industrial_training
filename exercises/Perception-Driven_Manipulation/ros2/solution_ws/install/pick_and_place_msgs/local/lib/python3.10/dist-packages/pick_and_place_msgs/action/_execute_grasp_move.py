# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExecuteGraspMove_Goal(type):
    """Metaclass of message 'ExecuteGraspMove_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PRE_GRASP': 1,
        'GRASP': 2,
        'RELEASE': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PRE_GRASP': cls.__constants['PRE_GRASP'],
            'GRASP': cls.__constants['GRASP'],
            'RELEASE': cls.__constants['RELEASE'],
        }

    @property
    def PRE_GRASP(self):
        """Message constant 'PRE_GRASP'."""
        return Metaclass_ExecuteGraspMove_Goal.__constants['PRE_GRASP']

    @property
    def GRASP(self):
        """Message constant 'GRASP'."""
        return Metaclass_ExecuteGraspMove_Goal.__constants['GRASP']

    @property
    def RELEASE(self):
        """Message constant 'RELEASE'."""
        return Metaclass_ExecuteGraspMove_Goal.__constants['RELEASE']


class ExecuteGraspMove_Goal(metaclass=Metaclass_ExecuteGraspMove_Goal):
    """
    Message class 'ExecuteGraspMove_Goal'.

    Constants:
      PRE_GRASP
      GRASP
      RELEASE
    """

    __slots__ = [
        '_goal',
        '_max_contact_force',
    ]

    _fields_and_field_types = {
        'goal': 'int32',
        'max_contact_force': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.goal = kwargs.get('goal', int())
        self.max_contact_force = kwargs.get('max_contact_force', float())

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
        if self.goal != other.goal:
            return False
        if self.max_contact_force != other.max_contact_force:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'goal' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'goal' field must be an integer in [-2147483648, 2147483647]"
        self._goal = value

    @builtins.property
    def max_contact_force(self):
        """Message field 'max_contact_force'."""
        return self._max_contact_force

    @max_contact_force.setter
    def max_contact_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_contact_force' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'max_contact_force' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._max_contact_force = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_Result(type):
    """Metaclass of message 'ExecuteGraspMove_Result'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_Result(metaclass=Metaclass_ExecuteGraspMove_Result):
    """Message class 'ExecuteGraspMove_Result'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_Feedback(type):
    """Metaclass of message 'ExecuteGraspMove_Feedback'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_Feedback(metaclass=Metaclass_ExecuteGraspMove_Feedback):
    """Message class 'ExecuteGraspMove_Feedback'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_SendGoal_Request(type):
    """Metaclass of message 'ExecuteGraspMove_SendGoal_Request'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__send_goal__request

            from pick_and_place_msgs.action import ExecuteGraspMove
            if ExecuteGraspMove.Goal.__class__._TYPE_SUPPORT is None:
                ExecuteGraspMove.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_SendGoal_Request(metaclass=Metaclass_ExecuteGraspMove_SendGoal_Request):
    """Message class 'ExecuteGraspMove_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'pick_and_place_msgs/ExecuteGraspMove_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['pick_and_place_msgs', 'action'], 'ExecuteGraspMove_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Goal
        self.goal = kwargs.get('goal', ExecuteGraspMove_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Goal
            assert \
                isinstance(value, ExecuteGraspMove_Goal), \
                "The 'goal' field must be a sub message of type 'ExecuteGraspMove_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_SendGoal_Response(type):
    """Metaclass of message 'ExecuteGraspMove_SendGoal_Response'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_SendGoal_Response(metaclass=Metaclass_ExecuteGraspMove_SendGoal_Response):
    """Message class 'ExecuteGraspMove_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_ExecuteGraspMove_SendGoal(type):
    """Metaclass of service 'ExecuteGraspMove_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__execute_grasp_move__send_goal

            from pick_and_place_msgs.action import _execute_grasp_move
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal_Request._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal_Request.__import_type_support__()
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal_Response._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal_Response.__import_type_support__()


class ExecuteGraspMove_SendGoal(metaclass=Metaclass_ExecuteGraspMove_SendGoal):
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_SendGoal_Request as Request
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_GetResult_Request(type):
    """Metaclass of message 'ExecuteGraspMove_GetResult_Request'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_GetResult_Request(metaclass=Metaclass_ExecuteGraspMove_GetResult_Request):
    """Message class 'ExecuteGraspMove_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_GetResult_Response(type):
    """Metaclass of message 'ExecuteGraspMove_GetResult_Response'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__get_result__response

            from pick_and_place_msgs.action import ExecuteGraspMove
            if ExecuteGraspMove.Result.__class__._TYPE_SUPPORT is None:
                ExecuteGraspMove.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_GetResult_Response(metaclass=Metaclass_ExecuteGraspMove_GetResult_Response):
    """Message class 'ExecuteGraspMove_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'pick_and_place_msgs/ExecuteGraspMove_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['pick_and_place_msgs', 'action'], 'ExecuteGraspMove_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Result
        self.result = kwargs.get('result', ExecuteGraspMove_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Result
            assert \
                isinstance(value, ExecuteGraspMove_Result), \
                "The 'result' field must be a sub message of type 'ExecuteGraspMove_Result'"
        self._result = value


class Metaclass_ExecuteGraspMove_GetResult(type):
    """Metaclass of service 'ExecuteGraspMove_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__execute_grasp_move__get_result

            from pick_and_place_msgs.action import _execute_grasp_move
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult_Request._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult_Request.__import_type_support__()
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult_Response._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult_Response.__import_type_support__()


class ExecuteGraspMove_GetResult(metaclass=Metaclass_ExecuteGraspMove_GetResult):
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_GetResult_Request as Request
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteGraspMove_FeedbackMessage(type):
    """Metaclass of message 'ExecuteGraspMove_FeedbackMessage'."""

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
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__execute_grasp_move__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__execute_grasp_move__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__execute_grasp_move__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__execute_grasp_move__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__execute_grasp_move__feedback_message

            from pick_and_place_msgs.action import ExecuteGraspMove
            if ExecuteGraspMove.Feedback.__class__._TYPE_SUPPORT is None:
                ExecuteGraspMove.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteGraspMove_FeedbackMessage(metaclass=Metaclass_ExecuteGraspMove_FeedbackMessage):
    """Message class 'ExecuteGraspMove_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'pick_and_place_msgs/ExecuteGraspMove_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['pick_and_place_msgs', 'action'], 'ExecuteGraspMove_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Feedback
        self.feedback = kwargs.get('feedback', ExecuteGraspMove_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Feedback
            assert \
                isinstance(value, ExecuteGraspMove_Feedback), \
                "The 'feedback' field must be a sub message of type 'ExecuteGraspMove_Feedback'"
        self._feedback = value


class Metaclass_ExecuteGraspMove(type):
    """Metaclass of action 'ExecuteGraspMove'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pick_and_place_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pick_and_place_msgs.action.ExecuteGraspMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__execute_grasp_move

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from pick_and_place_msgs.action import _execute_grasp_move
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_SendGoal.__import_type_support__()
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_GetResult.__import_type_support__()
            if _execute_grasp_move.Metaclass_ExecuteGraspMove_FeedbackMessage._TYPE_SUPPORT is None:
                _execute_grasp_move.Metaclass_ExecuteGraspMove_FeedbackMessage.__import_type_support__()


class ExecuteGraspMove(metaclass=Metaclass_ExecuteGraspMove):

    # The goal message defined in the action definition.
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Goal as Goal
    # The result message defined in the action definition.
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Result as Result
    # The feedback message defined in the action definition.
    from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from pick_and_place_msgs.action._execute_grasp_move import ExecuteGraspMove_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
