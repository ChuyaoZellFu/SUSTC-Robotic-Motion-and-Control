; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryErrorIdentifier.msg.html

(cl:defclass <TrajectoryErrorIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrajectoryErrorIdentifier (<TrajectoryErrorIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryErrorIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryErrorIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryErrorIdentifier> is deprecated: use kortex_driver-msg:TrajectoryErrorIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryErrorIdentifier>)))
    "Constants for message type '<TrajectoryErrorIdentifier>"
  '((:TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED . 0)
    (:TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE . 1)
    (:TRAJECTORY_ERROR_IDENTIFIER_TIME . 2)
    (:TRAJECTORY_ERROR_IDENTIFIER_POSITION . 3)
    (:TRAJECTORY_ERROR_IDENTIFIER_VELOCITY . 4)
    (:TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryErrorIdentifier)))
    "Constants for message type 'TrajectoryErrorIdentifier"
  '((:TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED . 0)
    (:TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE . 1)
    (:TRAJECTORY_ERROR_IDENTIFIER_TIME . 2)
    (:TRAJECTORY_ERROR_IDENTIFIER_POSITION . 3)
    (:TRAJECTORY_ERROR_IDENTIFIER_VELOCITY . 4)
    (:TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryErrorIdentifier>) ostream)
  "Serializes a message object of type '<TrajectoryErrorIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryErrorIdentifier>) istream)
  "Deserializes a message object of type '<TrajectoryErrorIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryErrorIdentifier>)))
  "Returns string type for a message object of type '<TrajectoryErrorIdentifier>"
  "kortex_driver/TrajectoryErrorIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryErrorIdentifier)))
  "Returns string type for a message object of type 'TrajectoryErrorIdentifier"
  "kortex_driver/TrajectoryErrorIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryErrorIdentifier>)))
  "Returns md5sum for a message object of type '<TrajectoryErrorIdentifier>"
  "d64304d18e3e5294df54e8c5ef2c83c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryErrorIdentifier)))
  "Returns md5sum for a message object of type 'TrajectoryErrorIdentifier"
  "d64304d18e3e5294df54e8c5ef2c83c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryErrorIdentifier>)))
  "Returns full string definition for message of type '<TrajectoryErrorIdentifier>"
  (cl:format cl:nil "~%uint32 TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE = 1~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_TIME = 2~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_POSITION = 3~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_VELOCITY = 4~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryErrorIdentifier)))
  "Returns full string definition for message of type 'TrajectoryErrorIdentifier"
  (cl:format cl:nil "~%uint32 TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE = 1~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_TIME = 2~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_POSITION = 3~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_VELOCITY = 4~%~%uint32 TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryErrorIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryErrorIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryErrorIdentifier
))
