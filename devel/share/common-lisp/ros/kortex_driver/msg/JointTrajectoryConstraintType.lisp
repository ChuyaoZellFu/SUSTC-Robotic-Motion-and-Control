; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointTrajectoryConstraintType.msg.html

(cl:defclass <JointTrajectoryConstraintType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JointTrajectoryConstraintType (<JointTrajectoryConstraintType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTrajectoryConstraintType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTrajectoryConstraintType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointTrajectoryConstraintType> is deprecated: use kortex_driver-msg:JointTrajectoryConstraintType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JointTrajectoryConstraintType>)))
    "Constants for message type '<JointTrajectoryConstraintType>"
  '((:UNSPECIFIED_JOINT_CONSTRAINT . 0)
    (:JOINT_CONSTRAINT_DURATION . 1)
    (:JOINT_CONSTRAINT_SPEED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JointTrajectoryConstraintType)))
    "Constants for message type 'JointTrajectoryConstraintType"
  '((:UNSPECIFIED_JOINT_CONSTRAINT . 0)
    (:JOINT_CONSTRAINT_DURATION . 1)
    (:JOINT_CONSTRAINT_SPEED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTrajectoryConstraintType>) ostream)
  "Serializes a message object of type '<JointTrajectoryConstraintType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTrajectoryConstraintType>) istream)
  "Deserializes a message object of type '<JointTrajectoryConstraintType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTrajectoryConstraintType>)))
  "Returns string type for a message object of type '<JointTrajectoryConstraintType>"
  "kortex_driver/JointTrajectoryConstraintType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTrajectoryConstraintType)))
  "Returns string type for a message object of type 'JointTrajectoryConstraintType"
  "kortex_driver/JointTrajectoryConstraintType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTrajectoryConstraintType>)))
  "Returns md5sum for a message object of type '<JointTrajectoryConstraintType>"
  "a007b68cae061ff813f21ddba74d4035")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTrajectoryConstraintType)))
  "Returns md5sum for a message object of type 'JointTrajectoryConstraintType"
  "a007b68cae061ff813f21ddba74d4035")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTrajectoryConstraintType>)))
  "Returns full string definition for message of type '<JointTrajectoryConstraintType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_JOINT_CONSTRAINT = 0~%~%uint32 JOINT_CONSTRAINT_DURATION = 1~%~%uint32 JOINT_CONSTRAINT_SPEED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTrajectoryConstraintType)))
  "Returns full string definition for message of type 'JointTrajectoryConstraintType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_JOINT_CONSTRAINT = 0~%~%uint32 JOINT_CONSTRAINT_DURATION = 1~%~%uint32 JOINT_CONSTRAINT_SPEED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTrajectoryConstraintType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTrajectoryConstraintType>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTrajectoryConstraintType
))
