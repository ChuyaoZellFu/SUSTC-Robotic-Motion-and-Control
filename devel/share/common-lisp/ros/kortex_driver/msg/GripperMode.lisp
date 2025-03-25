; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperMode.msg.html

(cl:defclass <GripperMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GripperMode (<GripperMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperMode> is deprecated: use kortex_driver-msg:GripperMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GripperMode>)))
    "Constants for message type '<GripperMode>"
  '((:UNSPECIFIED_GRIPPER_MODE . 0)
    (:GRIPPER_FORCE . 1)
    (:GRIPPER_SPEED . 2)
    (:GRIPPER_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GripperMode)))
    "Constants for message type 'GripperMode"
  '((:UNSPECIFIED_GRIPPER_MODE . 0)
    (:GRIPPER_FORCE . 1)
    (:GRIPPER_SPEED . 2)
    (:GRIPPER_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperMode>) ostream)
  "Serializes a message object of type '<GripperMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperMode>) istream)
  "Deserializes a message object of type '<GripperMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperMode>)))
  "Returns string type for a message object of type '<GripperMode>"
  "kortex_driver/GripperMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperMode)))
  "Returns string type for a message object of type 'GripperMode"
  "kortex_driver/GripperMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperMode>)))
  "Returns md5sum for a message object of type '<GripperMode>"
  "91237e67935408a1b67c1d3e83f53d6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperMode)))
  "Returns md5sum for a message object of type 'GripperMode"
  "91237e67935408a1b67c1d3e83f53d6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperMode>)))
  "Returns full string definition for message of type '<GripperMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GRIPPER_MODE = 0~%~%uint32 GRIPPER_FORCE = 1~%~%uint32 GRIPPER_SPEED = 2~%~%uint32 GRIPPER_POSITION = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperMode)))
  "Returns full string definition for message of type 'GripperMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GRIPPER_MODE = 0~%~%uint32 GRIPPER_FORCE = 1~%~%uint32 GRIPPER_SPEED = 2~%~%uint32 GRIPPER_POSITION = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperMode>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperMode
))
