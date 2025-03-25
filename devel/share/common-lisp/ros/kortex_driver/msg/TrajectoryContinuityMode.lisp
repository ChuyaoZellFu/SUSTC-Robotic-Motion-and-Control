; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryContinuityMode.msg.html

(cl:defclass <TrajectoryContinuityMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrajectoryContinuityMode (<TrajectoryContinuityMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryContinuityMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryContinuityMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryContinuityMode> is deprecated: use kortex_driver-msg:TrajectoryContinuityMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryContinuityMode>)))
    "Constants for message type '<TrajectoryContinuityMode>"
  '((:TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED . 0)
    (:TRAJECTORY_CONTINUITY_MODE_POSITION . 1)
    (:TRAJECTORY_CONTINUITY_MODE_SPEED . 2)
    (:TRAJECTORY_CONTINUITY_MODE_ACCELERATION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryContinuityMode)))
    "Constants for message type 'TrajectoryContinuityMode"
  '((:TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED . 0)
    (:TRAJECTORY_CONTINUITY_MODE_POSITION . 1)
    (:TRAJECTORY_CONTINUITY_MODE_SPEED . 2)
    (:TRAJECTORY_CONTINUITY_MODE_ACCELERATION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryContinuityMode>) ostream)
  "Serializes a message object of type '<TrajectoryContinuityMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryContinuityMode>) istream)
  "Deserializes a message object of type '<TrajectoryContinuityMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryContinuityMode>)))
  "Returns string type for a message object of type '<TrajectoryContinuityMode>"
  "kortex_driver/TrajectoryContinuityMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryContinuityMode)))
  "Returns string type for a message object of type 'TrajectoryContinuityMode"
  "kortex_driver/TrajectoryContinuityMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryContinuityMode>)))
  "Returns md5sum for a message object of type '<TrajectoryContinuityMode>"
  "a762cc9649feca6c4672c7308715b451")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryContinuityMode)))
  "Returns md5sum for a message object of type 'TrajectoryContinuityMode"
  "a762cc9649feca6c4672c7308715b451")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryContinuityMode>)))
  "Returns full string definition for message of type '<TrajectoryContinuityMode>"
  (cl:format cl:nil "~%uint32 TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_CONTINUITY_MODE_POSITION = 1~%~%uint32 TRAJECTORY_CONTINUITY_MODE_SPEED = 2~%~%uint32 TRAJECTORY_CONTINUITY_MODE_ACCELERATION = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryContinuityMode)))
  "Returns full string definition for message of type 'TrajectoryContinuityMode"
  (cl:format cl:nil "~%uint32 TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_CONTINUITY_MODE_POSITION = 1~%~%uint32 TRAJECTORY_CONTINUITY_MODE_SPEED = 2~%~%uint32 TRAJECTORY_CONTINUITY_MODE_ACCELERATION = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryContinuityMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryContinuityMode>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryContinuityMode
))
