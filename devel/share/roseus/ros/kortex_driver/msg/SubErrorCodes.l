;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SubErrorCodes)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SubErrorCodes (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SUBERRORCODES")
  (make-package "KORTEX_DRIVER::SUBERRORCODES"))

(in-package "ROS")
;;//! \htmlinclude SubErrorCodes.msg.html


(intern "*SUB_ERROR_NONE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*SUB_ERROR_NONE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*SUB_ERROR_NONE* 0)
(intern "*METHOD_FAILED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*METHOD_FAILED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*METHOD_FAILED* 1)
(intern "*UNIMPLEMENTED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNIMPLEMENTED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNIMPLEMENTED* 2)
(intern "*INVALID_PARAM*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INVALID_PARAM* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INVALID_PARAM* 3)
(intern "*UNSUPPORTED_SERVICE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_SERVICE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_SERVICE* 4)
(intern "*UNSUPPORTED_METHOD*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_METHOD* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_METHOD* 5)
(intern "*TOO_LARGE_ENCODED_FRAME_BUFFER*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*TOO_LARGE_ENCODED_FRAME_BUFFER* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*TOO_LARGE_ENCODED_FRAME_BUFFER* 6)
(intern "*FRAME_ENCODING_ERR*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*FRAME_ENCODING_ERR* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*FRAME_ENCODING_ERR* 7)
(intern "*FRAME_DECODING_ERR*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*FRAME_DECODING_ERR* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*FRAME_DECODING_ERR* 8)
(intern "*INCOMPATIBLE_HEADER_VERSION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INCOMPATIBLE_HEADER_VERSION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INCOMPATIBLE_HEADER_VERSION* 9)
(intern "*UNSUPPORTED_FRAME_TYPE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_FRAME_TYPE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_FRAME_TYPE* 10)
(intern "*UNREGISTERED_NOTIFICATION_RECEIVED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNREGISTERED_NOTIFICATION_RECEIVED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNREGISTERED_NOTIFICATION_RECEIVED* 11)
(intern "*INVALID_SESSION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INVALID_SESSION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INVALID_SESSION* 12)
(intern "*PAYLOAD_DECODING_ERR*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*PAYLOAD_DECODING_ERR* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*PAYLOAD_DECODING_ERR* 13)
(intern "*UNREGISTERED_FRAME_RECEIVED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNREGISTERED_FRAME_RECEIVED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNREGISTERED_FRAME_RECEIVED* 14)
(intern "*INVALID_PASSWORD*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INVALID_PASSWORD* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INVALID_PASSWORD* 15)
(intern "*USER_NOT_FOUND*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*USER_NOT_FOUND* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*USER_NOT_FOUND* 16)
(intern "*ENTITY_NOT_FOUND*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ENTITY_NOT_FOUND* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ENTITY_NOT_FOUND* 17)
(intern "*ROBOT_MOVEMENT_IN_PROGRESS*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_MOVEMENT_IN_PROGRESS* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_MOVEMENT_IN_PROGRESS* 18)
(intern "*ROBOT_NOT_MOVING*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_NOT_MOVING* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_NOT_MOVING* 19)
(intern "*NO_MORE_STORAGE_SPACE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*NO_MORE_STORAGE_SPACE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*NO_MORE_STORAGE_SPACE* 20)
(intern "*ROBOT_NOT_READY*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_NOT_READY* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_NOT_READY* 21)
(intern "*ROBOT_IN_FAULT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_IN_FAULT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_IN_FAULT* 22)
(intern "*ROBOT_IN_MAINTENANCE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_IN_MAINTENANCE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_IN_MAINTENANCE* 23)
(intern "*ROBOT_IN_UPDATE_MODE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_IN_UPDATE_MODE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_IN_UPDATE_MODE* 24)
(intern "*ROBOT_IN_EMERGENCY_STOP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROBOT_IN_EMERGENCY_STOP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROBOT_IN_EMERGENCY_STOP* 25)
(intern "*SINGLE_LEVEL_SERVOING*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*SINGLE_LEVEL_SERVOING* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*SINGLE_LEVEL_SERVOING* 26)
(intern "*LOW_LEVEL_SERVOING*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*LOW_LEVEL_SERVOING* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*LOW_LEVEL_SERVOING* 27)
(intern "*MAPPING_GROUP_NON_ROOT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAPPING_GROUP_NON_ROOT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAPPING_GROUP_NON_ROOT* 28)
(intern "*MAPPING_INVALID_GROUP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAPPING_INVALID_GROUP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAPPING_INVALID_GROUP* 29)
(intern "*MAPPING_INVALID_MAP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAPPING_INVALID_MAP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAPPING_INVALID_MAP* 30)
(intern "*MAP_GROUP_INVALID_MAP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAP_GROUP_INVALID_MAP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_MAP* 31)
(intern "*MAP_GROUP_INVALID_PARENT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAP_GROUP_INVALID_PARENT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_PARENT* 32)
(intern "*MAP_GROUP_INVALID_CHILD*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAP_GROUP_INVALID_CHILD* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_CHILD* 33)
(intern "*MAP_GROUP_INVALID_MOVE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAP_GROUP_INVALID_MOVE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_MOVE* 34)
(intern "*MAP_IN_USE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAP_IN_USE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAP_IN_USE* 35)
(intern "*WIFI_CONNECT_ERROR*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*WIFI_CONNECT_ERROR* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*WIFI_CONNECT_ERROR* 36)
(intern "*UNSUPPORTED_NETWORK_TYPE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_NETWORK_TYPE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_NETWORK_TYPE* 37)
(intern "*TOO_LARGE_ENCODED_PAYLOAD_BUFFER*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*TOO_LARGE_ENCODED_PAYLOAD_BUFFER* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*TOO_LARGE_ENCODED_PAYLOAD_BUFFER* 38)
(intern "*UPDATE_PERMISSION_DENIED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UPDATE_PERMISSION_DENIED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UPDATE_PERMISSION_DENIED* 39)
(intern "*DELETE_PERMISSION_DENIED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*DELETE_PERMISSION_DENIED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*DELETE_PERMISSION_DENIED* 40)
(intern "*DATABASE_ERROR*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*DATABASE_ERROR* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*DATABASE_ERROR* 41)
(intern "*UNSUPPORTED_OPTION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_OPTION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_OPTION* 42)
(intern "*UNSUPPORTED_RESOLUTION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_RESOLUTION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_RESOLUTION* 43)
(intern "*UNSUPPORTED_FRAME_RATE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_FRAME_RATE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_FRAME_RATE* 44)
(intern "*UNSUPPORTED_BIT_RATE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_BIT_RATE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_BIT_RATE* 45)
(intern "*UNSUPPORTED_ACTION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_ACTION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_ACTION* 46)
(intern "*UNSUPPORTED_FOCUS_ACTION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_FOCUS_ACTION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_FOCUS_ACTION* 47)
(intern "*VALUE_IS_ABOVE_MAXIMUM*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*VALUE_IS_ABOVE_MAXIMUM* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*VALUE_IS_ABOVE_MAXIMUM* 48)
(intern "*VALUE_IS_BELOW_MINIMUM*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*VALUE_IS_BELOW_MINIMUM* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*VALUE_IS_BELOW_MINIMUM* 49)
(intern "*DEVICE_DISCONNECTED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*DEVICE_DISCONNECTED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*DEVICE_DISCONNECTED* 50)
(intern "*DEVICE_NOT_READY*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*DEVICE_NOT_READY* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*DEVICE_NOT_READY* 51)
(intern "*INVALID_DEVICE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INVALID_DEVICE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INVALID_DEVICE* 52)
(intern "*SAFETY_THRESHOLD_REACHED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*SAFETY_THRESHOLD_REACHED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*SAFETY_THRESHOLD_REACHED* 53)
(intern "*INVALID_USER_SESSION_ACCESS*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*INVALID_USER_SESSION_ACCESS* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*INVALID_USER_SESSION_ACCESS* 54)
(intern "*CONTROL_MANUAL_STOP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_MANUAL_STOP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_MANUAL_STOP* 55)
(intern "*CONTROL_OUTSIDE_WORKSPACE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_OUTSIDE_WORKSPACE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_OUTSIDE_WORKSPACE* 56)
(intern "*CONTROL_ACTUATOR_COUNT_MISMATCH*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_ACTUATOR_COUNT_MISMATCH* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_ACTUATOR_COUNT_MISMATCH* 57)
(intern "*CONTROL_INVALID_DURATION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_INVALID_DURATION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_INVALID_DURATION* 58)
(intern "*CONTROL_INVALID_SPEED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_INVALID_SPEED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_INVALID_SPEED* 59)
(intern "*CONTROL_LARGE_SPEED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_LARGE_SPEED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_LARGE_SPEED* 60)
(intern "*CONTROL_INVALID_ACCELERATION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_INVALID_ACCELERATION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_INVALID_ACCELERATION* 61)
(intern "*CONTROL_INVALID_TIME_STEP*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_INVALID_TIME_STEP* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_INVALID_TIME_STEP* 62)
(intern "*CONTROL_LARGE_SIZE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_LARGE_SIZE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_LARGE_SIZE* 63)
(intern "*CONTROL_WRONG_MODE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_WRONG_MODE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_WRONG_MODE* 64)
(intern "*CONTROL_JOINT_POSITION_LIMIT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_JOINT_POSITION_LIMIT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_JOINT_POSITION_LIMIT* 65)
(intern "*CONTROL_NO_FILE_IN_MEMORY*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_NO_FILE_IN_MEMORY* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_NO_FILE_IN_MEMORY* 66)
(intern "*CONTROL_INDEX_OUT_OF_TRAJECTORY*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_INDEX_OUT_OF_TRAJECTORY* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_INDEX_OUT_OF_TRAJECTORY* 67)
(intern "*CONTROL_ALREADY_RUNNING*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_ALREADY_RUNNING* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_ALREADY_RUNNING* 68)
(intern "*CONTROL_WRONG_STARTING_POINT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_WRONG_STARTING_POINT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_WRONG_STARTING_POINT* 69)
(intern "*CONTROL_CARTESIAN_CANNOT_START*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_CARTESIAN_CANNOT_START* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_CARTESIAN_CANNOT_START* 70)
(intern "*CONTROL_UNDEFINED_CONSTRAINT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_UNDEFINED_CONSTRAINT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_UNDEFINED_CONSTRAINT* 71)
(intern "*CONTROL_UNINITIALIZED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_UNINITIALIZED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_UNINITIALIZED* 72)
(intern "*CONTROL_NO_ACTION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_NO_ACTION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_NO_ACTION* 73)
(intern "*CONTROL_UNDEFINED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_UNDEFINED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_UNDEFINED* 74)
(intern "*WRONG_SERVOING_MODE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*WRONG_SERVOING_MODE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*WRONG_SERVOING_MODE* 75)
(intern "*CONTROL_WRONG_STARTING_SPEED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_WRONG_STARTING_SPEED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_WRONG_STARTING_SPEED* 76)
(intern "*USERNAME_LENGTH_EXCEEDED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*USERNAME_LENGTH_EXCEEDED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*USERNAME_LENGTH_EXCEEDED* 100)
(intern "*FIRSTNAME_LENGTH_EXCEEDED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*FIRSTNAME_LENGTH_EXCEEDED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*FIRSTNAME_LENGTH_EXCEEDED* 101)
(intern "*LASTNAME_LENGTH_EXCEEDED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*LASTNAME_LENGTH_EXCEEDED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*LASTNAME_LENGTH_EXCEEDED* 102)
(intern "*PASSWORD_LENGTH_EXCEEDED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*PASSWORD_LENGTH_EXCEEDED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*PASSWORD_LENGTH_EXCEEDED* 103)
(intern "*USERNAME_ALREADY_EXISTS*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*USERNAME_ALREADY_EXISTS* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*USERNAME_ALREADY_EXISTS* 104)
(intern "*USERNAME_EMPTY*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*USERNAME_EMPTY* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*USERNAME_EMPTY* 105)
(intern "*PASSWORD_NOT_CHANGED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*PASSWORD_NOT_CHANGED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*PASSWORD_NOT_CHANGED* 106)
(intern "*MAXIMUM_USER_PROFILES_USED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*MAXIMUM_USER_PROFILES_USED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*MAXIMUM_USER_PROFILES_USED* 107)
(intern "*ROUTER_UNVAILABLE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ROUTER_UNVAILABLE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ROUTER_UNVAILABLE* 108)
(intern "*ADDRESS_NOT_IN_VALID_RANGE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ADDRESS_NOT_IN_VALID_RANGE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ADDRESS_NOT_IN_VALID_RANGE* 120)
(intern "*ADDRESS_NOT_CONFIGURABLE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ADDRESS_NOT_CONFIGURABLE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ADDRESS_NOT_CONFIGURABLE* 121)
(intern "*SESSION_NOT_IN_CONTROL*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*SESSION_NOT_IN_CONTROL* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*SESSION_NOT_IN_CONTROL* 130)
(intern "*METHOD_TIMEOUT*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*METHOD_TIMEOUT* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*METHOD_TIMEOUT* 131)
(intern "*UNSUPPORTED_ROBOT_CONFIGURATION*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*UNSUPPORTED_ROBOT_CONFIGURATION* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*UNSUPPORTED_ROBOT_CONFIGURATION* 132)
(intern "*NVRAM_READ_FAIL*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*NVRAM_READ_FAIL* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*NVRAM_READ_FAIL* 133)
(intern "*NVRAM_WRITE_FAIL*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*NVRAM_WRITE_FAIL* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*NVRAM_WRITE_FAIL* 134)
(intern "*NETWORK_NO_ADDRESS_ASSIGNED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*NETWORK_NO_ADDRESS_ASSIGNED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*NETWORK_NO_ADDRESS_ASSIGNED* 135)
(intern "*READ_PERMISSION_DENIED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*READ_PERMISSION_DENIED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*READ_PERMISSION_DENIED* 136)
(intern "*CONTROLLER_INVALID_MAPPING*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROLLER_INVALID_MAPPING* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROLLER_INVALID_MAPPING* 137)
(intern "*ACTION_IN_USE*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*ACTION_IN_USE* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*ACTION_IN_USE* 138)
(intern "*SEND_FAILED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*SEND_FAILED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*SEND_FAILED* 139)
(intern "*CONTROL_WAYPOINT_TRAJECTORY_ABORTED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_WAYPOINT_TRAJECTORY_ABORTED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_WAYPOINT_TRAJECTORY_ABORTED* 140)
(intern "*CONTROL_PERMISSION_DENIED*" (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(shadow '*CONTROL_PERMISSION_DENIED* (find-package "KORTEX_DRIVER::SUBERRORCODES"))
(defconstant kortex_driver::SubErrorCodes::*CONTROL_PERMISSION_DENIED* 141)

(defun kortex_driver::SubErrorCodes-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::SubErrorCodes::*SUB_ERROR_NONE*)
        ((= const 1) 'kortex_driver::SubErrorCodes::*METHOD_FAILED*)
        ((= const 2) 'kortex_driver::SubErrorCodes::*UNIMPLEMENTED*)
        ((= const 3) 'kortex_driver::SubErrorCodes::*INVALID_PARAM*)
        ((= const 4) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_SERVICE*)
        ((= const 5) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_METHOD*)
        ((= const 6) 'kortex_driver::SubErrorCodes::*TOO_LARGE_ENCODED_FRAME_BUFFER*)
        ((= const 7) 'kortex_driver::SubErrorCodes::*FRAME_ENCODING_ERR*)
        ((= const 8) 'kortex_driver::SubErrorCodes::*FRAME_DECODING_ERR*)
        ((= const 9) 'kortex_driver::SubErrorCodes::*INCOMPATIBLE_HEADER_VERSION*)
        ((= const 10) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_FRAME_TYPE*)
        ((= const 11) 'kortex_driver::SubErrorCodes::*UNREGISTERED_NOTIFICATION_RECEIVED*)
        ((= const 12) 'kortex_driver::SubErrorCodes::*INVALID_SESSION*)
        ((= const 13) 'kortex_driver::SubErrorCodes::*PAYLOAD_DECODING_ERR*)
        ((= const 14) 'kortex_driver::SubErrorCodes::*UNREGISTERED_FRAME_RECEIVED*)
        ((= const 15) 'kortex_driver::SubErrorCodes::*INVALID_PASSWORD*)
        ((= const 16) 'kortex_driver::SubErrorCodes::*USER_NOT_FOUND*)
        ((= const 17) 'kortex_driver::SubErrorCodes::*ENTITY_NOT_FOUND*)
        ((= const 18) 'kortex_driver::SubErrorCodes::*ROBOT_MOVEMENT_IN_PROGRESS*)
        ((= const 19) 'kortex_driver::SubErrorCodes::*ROBOT_NOT_MOVING*)
        ((= const 20) 'kortex_driver::SubErrorCodes::*NO_MORE_STORAGE_SPACE*)
        ((= const 21) 'kortex_driver::SubErrorCodes::*ROBOT_NOT_READY*)
        ((= const 22) 'kortex_driver::SubErrorCodes::*ROBOT_IN_FAULT*)
        ((= const 23) 'kortex_driver::SubErrorCodes::*ROBOT_IN_MAINTENANCE*)
        ((= const 24) 'kortex_driver::SubErrorCodes::*ROBOT_IN_UPDATE_MODE*)
        ((= const 25) 'kortex_driver::SubErrorCodes::*ROBOT_IN_EMERGENCY_STOP*)
        ((= const 26) 'kortex_driver::SubErrorCodes::*SINGLE_LEVEL_SERVOING*)
        ((= const 27) 'kortex_driver::SubErrorCodes::*LOW_LEVEL_SERVOING*)
        ((= const 28) 'kortex_driver::SubErrorCodes::*MAPPING_GROUP_NON_ROOT*)
        ((= const 29) 'kortex_driver::SubErrorCodes::*MAPPING_INVALID_GROUP*)
        ((= const 30) 'kortex_driver::SubErrorCodes::*MAPPING_INVALID_MAP*)
        ((= const 31) 'kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_MAP*)
        ((= const 32) 'kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_PARENT*)
        ((= const 33) 'kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_CHILD*)
        ((= const 34) 'kortex_driver::SubErrorCodes::*MAP_GROUP_INVALID_MOVE*)
        ((= const 35) 'kortex_driver::SubErrorCodes::*MAP_IN_USE*)
        ((= const 36) 'kortex_driver::SubErrorCodes::*WIFI_CONNECT_ERROR*)
        ((= const 37) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_NETWORK_TYPE*)
        ((= const 38) 'kortex_driver::SubErrorCodes::*TOO_LARGE_ENCODED_PAYLOAD_BUFFER*)
        ((= const 39) 'kortex_driver::SubErrorCodes::*UPDATE_PERMISSION_DENIED*)
        ((= const 40) 'kortex_driver::SubErrorCodes::*DELETE_PERMISSION_DENIED*)
        ((= const 41) 'kortex_driver::SubErrorCodes::*DATABASE_ERROR*)
        ((= const 42) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_OPTION*)
        ((= const 43) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_RESOLUTION*)
        ((= const 44) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_FRAME_RATE*)
        ((= const 45) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_BIT_RATE*)
        ((= const 46) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_ACTION*)
        ((= const 47) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_FOCUS_ACTION*)
        ((= const 48) 'kortex_driver::SubErrorCodes::*VALUE_IS_ABOVE_MAXIMUM*)
        ((= const 49) 'kortex_driver::SubErrorCodes::*VALUE_IS_BELOW_MINIMUM*)
        ((= const 50) 'kortex_driver::SubErrorCodes::*DEVICE_DISCONNECTED*)
        ((= const 51) 'kortex_driver::SubErrorCodes::*DEVICE_NOT_READY*)
        ((= const 52) 'kortex_driver::SubErrorCodes::*INVALID_DEVICE*)
        ((= const 53) 'kortex_driver::SubErrorCodes::*SAFETY_THRESHOLD_REACHED*)
        ((= const 54) 'kortex_driver::SubErrorCodes::*INVALID_USER_SESSION_ACCESS*)
        ((= const 55) 'kortex_driver::SubErrorCodes::*CONTROL_MANUAL_STOP*)
        ((= const 56) 'kortex_driver::SubErrorCodes::*CONTROL_OUTSIDE_WORKSPACE*)
        ((= const 57) 'kortex_driver::SubErrorCodes::*CONTROL_ACTUATOR_COUNT_MISMATCH*)
        ((= const 58) 'kortex_driver::SubErrorCodes::*CONTROL_INVALID_DURATION*)
        ((= const 59) 'kortex_driver::SubErrorCodes::*CONTROL_INVALID_SPEED*)
        ((= const 60) 'kortex_driver::SubErrorCodes::*CONTROL_LARGE_SPEED*)
        ((= const 61) 'kortex_driver::SubErrorCodes::*CONTROL_INVALID_ACCELERATION*)
        ((= const 62) 'kortex_driver::SubErrorCodes::*CONTROL_INVALID_TIME_STEP*)
        ((= const 63) 'kortex_driver::SubErrorCodes::*CONTROL_LARGE_SIZE*)
        ((= const 64) 'kortex_driver::SubErrorCodes::*CONTROL_WRONG_MODE*)
        ((= const 65) 'kortex_driver::SubErrorCodes::*CONTROL_JOINT_POSITION_LIMIT*)
        ((= const 66) 'kortex_driver::SubErrorCodes::*CONTROL_NO_FILE_IN_MEMORY*)
        ((= const 67) 'kortex_driver::SubErrorCodes::*CONTROL_INDEX_OUT_OF_TRAJECTORY*)
        ((= const 68) 'kortex_driver::SubErrorCodes::*CONTROL_ALREADY_RUNNING*)
        ((= const 69) 'kortex_driver::SubErrorCodes::*CONTROL_WRONG_STARTING_POINT*)
        ((= const 70) 'kortex_driver::SubErrorCodes::*CONTROL_CARTESIAN_CANNOT_START*)
        ((= const 71) 'kortex_driver::SubErrorCodes::*CONTROL_UNDEFINED_CONSTRAINT*)
        ((= const 72) 'kortex_driver::SubErrorCodes::*CONTROL_UNINITIALIZED*)
        ((= const 73) 'kortex_driver::SubErrorCodes::*CONTROL_NO_ACTION*)
        ((= const 74) 'kortex_driver::SubErrorCodes::*CONTROL_UNDEFINED*)
        ((= const 75) 'kortex_driver::SubErrorCodes::*WRONG_SERVOING_MODE*)
        ((= const 76) 'kortex_driver::SubErrorCodes::*CONTROL_WRONG_STARTING_SPEED*)
        ((= const 100) 'kortex_driver::SubErrorCodes::*USERNAME_LENGTH_EXCEEDED*)
        ((= const 101) 'kortex_driver::SubErrorCodes::*FIRSTNAME_LENGTH_EXCEEDED*)
        ((= const 102) 'kortex_driver::SubErrorCodes::*LASTNAME_LENGTH_EXCEEDED*)
        ((= const 103) 'kortex_driver::SubErrorCodes::*PASSWORD_LENGTH_EXCEEDED*)
        ((= const 104) 'kortex_driver::SubErrorCodes::*USERNAME_ALREADY_EXISTS*)
        ((= const 105) 'kortex_driver::SubErrorCodes::*USERNAME_EMPTY*)
        ((= const 106) 'kortex_driver::SubErrorCodes::*PASSWORD_NOT_CHANGED*)
        ((= const 107) 'kortex_driver::SubErrorCodes::*MAXIMUM_USER_PROFILES_USED*)
        ((= const 108) 'kortex_driver::SubErrorCodes::*ROUTER_UNVAILABLE*)
        ((= const 120) 'kortex_driver::SubErrorCodes::*ADDRESS_NOT_IN_VALID_RANGE*)
        ((= const 121) 'kortex_driver::SubErrorCodes::*ADDRESS_NOT_CONFIGURABLE*)
        ((= const 130) 'kortex_driver::SubErrorCodes::*SESSION_NOT_IN_CONTROL*)
        ((= const 131) 'kortex_driver::SubErrorCodes::*METHOD_TIMEOUT*)
        ((= const 132) 'kortex_driver::SubErrorCodes::*UNSUPPORTED_ROBOT_CONFIGURATION*)
        ((= const 133) 'kortex_driver::SubErrorCodes::*NVRAM_READ_FAIL*)
        ((= const 134) 'kortex_driver::SubErrorCodes::*NVRAM_WRITE_FAIL*)
        ((= const 135) 'kortex_driver::SubErrorCodes::*NETWORK_NO_ADDRESS_ASSIGNED*)
        ((= const 136) 'kortex_driver::SubErrorCodes::*READ_PERMISSION_DENIED*)
        ((= const 137) 'kortex_driver::SubErrorCodes::*CONTROLLER_INVALID_MAPPING*)
        ((= const 138) 'kortex_driver::SubErrorCodes::*ACTION_IN_USE*)
        ((= const 139) 'kortex_driver::SubErrorCodes::*SEND_FAILED*)
        ((= const 140) 'kortex_driver::SubErrorCodes::*CONTROL_WAYPOINT_TRAJECTORY_ABORTED*)
        ((= const 141) 'kortex_driver::SubErrorCodes::*CONTROL_PERMISSION_DENIED*)
        (t nil)))

(defclass kortex_driver::SubErrorCodes
  :super ros::object
  :slots ())

(defmethod kortex_driver::SubErrorCodes
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::SubErrorCodes :md5sum-) "498c5ef0d083804e5f4c80a885f37133")
(setf (get kortex_driver::SubErrorCodes :datatype-) "kortex_driver/SubErrorCodes")
(setf (get kortex_driver::SubErrorCodes :definition-)
      "
uint32 SUB_ERROR_NONE = 0

uint32 METHOD_FAILED = 1

uint32 UNIMPLEMENTED = 2

uint32 INVALID_PARAM = 3

uint32 UNSUPPORTED_SERVICE = 4

uint32 UNSUPPORTED_METHOD = 5

uint32 TOO_LARGE_ENCODED_FRAME_BUFFER = 6

uint32 FRAME_ENCODING_ERR = 7

uint32 FRAME_DECODING_ERR = 8

uint32 INCOMPATIBLE_HEADER_VERSION = 9

uint32 UNSUPPORTED_FRAME_TYPE = 10

uint32 UNREGISTERED_NOTIFICATION_RECEIVED = 11

uint32 INVALID_SESSION = 12

uint32 PAYLOAD_DECODING_ERR = 13

uint32 UNREGISTERED_FRAME_RECEIVED = 14

uint32 INVALID_PASSWORD = 15

uint32 USER_NOT_FOUND = 16

uint32 ENTITY_NOT_FOUND = 17

uint32 ROBOT_MOVEMENT_IN_PROGRESS = 18

uint32 ROBOT_NOT_MOVING = 19

uint32 NO_MORE_STORAGE_SPACE = 20

uint32 ROBOT_NOT_READY = 21

uint32 ROBOT_IN_FAULT = 22

uint32 ROBOT_IN_MAINTENANCE = 23

uint32 ROBOT_IN_UPDATE_MODE = 24

uint32 ROBOT_IN_EMERGENCY_STOP = 25

uint32 SINGLE_LEVEL_SERVOING = 26

uint32 LOW_LEVEL_SERVOING = 27

uint32 MAPPING_GROUP_NON_ROOT = 28

uint32 MAPPING_INVALID_GROUP = 29

uint32 MAPPING_INVALID_MAP = 30

uint32 MAP_GROUP_INVALID_MAP = 31

uint32 MAP_GROUP_INVALID_PARENT = 32

uint32 MAP_GROUP_INVALID_CHILD = 33

uint32 MAP_GROUP_INVALID_MOVE = 34

uint32 MAP_IN_USE = 35

uint32 WIFI_CONNECT_ERROR = 36

uint32 UNSUPPORTED_NETWORK_TYPE = 37

uint32 TOO_LARGE_ENCODED_PAYLOAD_BUFFER = 38

uint32 UPDATE_PERMISSION_DENIED = 39

uint32 DELETE_PERMISSION_DENIED = 40

uint32 DATABASE_ERROR = 41

uint32 UNSUPPORTED_OPTION = 42

uint32 UNSUPPORTED_RESOLUTION = 43

uint32 UNSUPPORTED_FRAME_RATE = 44

uint32 UNSUPPORTED_BIT_RATE = 45

uint32 UNSUPPORTED_ACTION = 46

uint32 UNSUPPORTED_FOCUS_ACTION = 47

uint32 VALUE_IS_ABOVE_MAXIMUM = 48

uint32 VALUE_IS_BELOW_MINIMUM = 49

uint32 DEVICE_DISCONNECTED = 50

uint32 DEVICE_NOT_READY = 51

uint32 INVALID_DEVICE = 52

uint32 SAFETY_THRESHOLD_REACHED = 53

uint32 INVALID_USER_SESSION_ACCESS = 54

uint32 CONTROL_MANUAL_STOP = 55

uint32 CONTROL_OUTSIDE_WORKSPACE = 56

uint32 CONTROL_ACTUATOR_COUNT_MISMATCH = 57

uint32 CONTROL_INVALID_DURATION = 58

uint32 CONTROL_INVALID_SPEED = 59

uint32 CONTROL_LARGE_SPEED = 60

uint32 CONTROL_INVALID_ACCELERATION = 61

uint32 CONTROL_INVALID_TIME_STEP = 62

uint32 CONTROL_LARGE_SIZE = 63

uint32 CONTROL_WRONG_MODE = 64

uint32 CONTROL_JOINT_POSITION_LIMIT = 65

uint32 CONTROL_NO_FILE_IN_MEMORY = 66

uint32 CONTROL_INDEX_OUT_OF_TRAJECTORY = 67

uint32 CONTROL_ALREADY_RUNNING = 68

uint32 CONTROL_WRONG_STARTING_POINT = 69

uint32 CONTROL_CARTESIAN_CANNOT_START = 70

uint32 CONTROL_UNDEFINED_CONSTRAINT = 71

uint32 CONTROL_UNINITIALIZED = 72

uint32 CONTROL_NO_ACTION = 73

uint32 CONTROL_UNDEFINED = 74

uint32 WRONG_SERVOING_MODE = 75

uint32 CONTROL_WRONG_STARTING_SPEED = 76

uint32 USERNAME_LENGTH_EXCEEDED = 100

uint32 FIRSTNAME_LENGTH_EXCEEDED = 101

uint32 LASTNAME_LENGTH_EXCEEDED = 102

uint32 PASSWORD_LENGTH_EXCEEDED = 103

uint32 USERNAME_ALREADY_EXISTS = 104

uint32 USERNAME_EMPTY = 105

uint32 PASSWORD_NOT_CHANGED = 106

uint32 MAXIMUM_USER_PROFILES_USED = 107

uint32 ROUTER_UNVAILABLE = 108

uint32 ADDRESS_NOT_IN_VALID_RANGE = 120

uint32 ADDRESS_NOT_CONFIGURABLE = 121

uint32 SESSION_NOT_IN_CONTROL = 130

uint32 METHOD_TIMEOUT = 131

uint32 UNSUPPORTED_ROBOT_CONFIGURATION = 132

uint32 NVRAM_READ_FAIL = 133

uint32 NVRAM_WRITE_FAIL = 134

uint32 NETWORK_NO_ADDRESS_ASSIGNED = 135

uint32 READ_PERMISSION_DENIED = 136

uint32 CONTROLLER_INVALID_MAPPING = 137

uint32 ACTION_IN_USE = 138

uint32 SEND_FAILED = 139

uint32 CONTROL_WAYPOINT_TRAJECTORY_ABORTED = 140

uint32 CONTROL_PERMISSION_DENIED = 141

")



(provide :kortex_driver/SubErrorCodes "498c5ef0d083804e5f4c80a885f37133")


