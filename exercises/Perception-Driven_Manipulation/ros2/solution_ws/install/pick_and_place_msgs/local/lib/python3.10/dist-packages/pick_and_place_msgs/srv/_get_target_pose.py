# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pick_and_place_msgs:srv/GetTargetPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTargetPose_Request(type):
    """Metaclass of message 'GetTargetPose_Request'."""

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
                'pick_and_place_msgs.srv.GetTargetPose_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_target_pose__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_target_pose__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_target_pose__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_target_pose__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_target_pose__request

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from shape_msgs.msg import SolidPrimitive
            if SolidPrimitive.__class__._TYPE_SUPPORT is None:
                SolidPrimitive.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTargetPose_Request(metaclass=Metaclass_GetTargetPose_Request):
    """Message class 'GetTargetPose_Request'."""

    __slots__ = [
        '_world_frame_id',
        '_ar_tag_frame_id',
        '_shape',
        '_remove_at_poses',
    ]

    _fields_and_field_types = {
        'world_frame_id': 'string',
        'ar_tag_frame_id': 'string',
        'shape': 'shape_msgs/SolidPrimitive',
        'remove_at_poses': 'sequence<geometry_msgs/Pose>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['shape_msgs', 'msg'], 'SolidPrimitive'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.world_frame_id = kwargs.get('world_frame_id', str())
        self.ar_tag_frame_id = kwargs.get('ar_tag_frame_id', str())
        from shape_msgs.msg import SolidPrimitive
        self.shape = kwargs.get('shape', SolidPrimitive())
        self.remove_at_poses = kwargs.get('remove_at_poses', [])

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
        if self.world_frame_id != other.world_frame_id:
            return False
        if self.ar_tag_frame_id != other.ar_tag_frame_id:
            return False
        if self.shape != other.shape:
            return False
        if self.remove_at_poses != other.remove_at_poses:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def world_frame_id(self):
        """Message field 'world_frame_id'."""
        return self._world_frame_id

    @world_frame_id.setter
    def world_frame_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'world_frame_id' field must be of type 'str'"
        self._world_frame_id = value

    @builtins.property
    def ar_tag_frame_id(self):
        """Message field 'ar_tag_frame_id'."""
        return self._ar_tag_frame_id

    @ar_tag_frame_id.setter
    def ar_tag_frame_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ar_tag_frame_id' field must be of type 'str'"
        self._ar_tag_frame_id = value

    @builtins.property
    def shape(self):
        """Message field 'shape'."""
        return self._shape

    @shape.setter
    def shape(self, value):
        if __debug__:
            from shape_msgs.msg import SolidPrimitive
            assert \
                isinstance(value, SolidPrimitive), \
                "The 'shape' field must be a sub message of type 'SolidPrimitive'"
        self._shape = value

    @builtins.property
    def remove_at_poses(self):
        """Message field 'remove_at_poses'."""
        return self._remove_at_poses

    @remove_at_poses.setter
    def remove_at_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
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
                 all(isinstance(v, Pose) for v in value) and
                 True), \
                "The 'remove_at_poses' field must be a set or sequence and each value of type 'Pose'"
        self._remove_at_poses = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTargetPose_Response(type):
    """Metaclass of message 'GetTargetPose_Response'."""

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
                'pick_and_place_msgs.srv.GetTargetPose_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_target_pose__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_target_pose__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_target_pose__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_target_pose__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_target_pose__response

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTargetPose_Response(metaclass=Metaclass_GetTargetPose_Response):
    """Message class 'GetTargetPose_Response'."""

    __slots__ = [
        '_succeeded',
        '_target_pose',
    ]

    _fields_and_field_types = {
        'succeeded': 'boolean',
        'target_pose': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.succeeded = kwargs.get('succeeded', bool())
        from geometry_msgs.msg import Pose
        self.target_pose = kwargs.get('target_pose', Pose())

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
        if self.succeeded != other.succeeded:
            return False
        if self.target_pose != other.target_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def succeeded(self):
        """Message field 'succeeded'."""
        return self._succeeded

    @succeeded.setter
    def succeeded(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'succeeded' field must be of type 'bool'"
        self._succeeded = value

    @builtins.property
    def target_pose(self):
        """Message field 'target_pose'."""
        return self._target_pose

    @target_pose.setter
    def target_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'target_pose' field must be a sub message of type 'Pose'"
        self._target_pose = value


class Metaclass_GetTargetPose(type):
    """Metaclass of service 'GetTargetPose'."""

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
                'pick_and_place_msgs.srv.GetTargetPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_target_pose

            from pick_and_place_msgs.srv import _get_target_pose
            if _get_target_pose.Metaclass_GetTargetPose_Request._TYPE_SUPPORT is None:
                _get_target_pose.Metaclass_GetTargetPose_Request.__import_type_support__()
            if _get_target_pose.Metaclass_GetTargetPose_Response._TYPE_SUPPORT is None:
                _get_target_pose.Metaclass_GetTargetPose_Response.__import_type_support__()


class GetTargetPose(metaclass=Metaclass_GetTargetPose):
    from pick_and_place_msgs.srv._get_target_pose import GetTargetPose_Request as Request
    from pick_and_place_msgs.srv._get_target_pose import GetTargetPose_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
