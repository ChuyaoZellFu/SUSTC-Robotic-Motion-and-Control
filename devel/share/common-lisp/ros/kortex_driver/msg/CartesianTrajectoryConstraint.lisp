; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianTrajectoryConstraint.msg.html

(cl:defclass <CartesianTrajectoryConstraint> (roslisp-msg-protocol:ros-message)
  ((oneof_type
    :reader oneof_type
    :initarg :oneof_type
    :type kortex_driver-msg:CartesianTrajectoryConstraint_type
    :initform (cl:make-instance 'kortex_driver-msg:CartesianTrajectoryConstraint_type)))
)

(cl:defclass CartesianTrajectoryConstraint (<CartesianTrajectoryConstraint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianTrajectoryConstraint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianTrajectoryConstraint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianTrajectoryConstraint> is deprecated: use kortex_driver-msg:CartesianTrajectoryConstraint instead.")))

(cl:ensure-generic-function 'oneof_type-val :lambda-list '(m))
(cl:defmethod oneof_type-val ((m <CartesianTrajectoryConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_type-val is deprecated.  Use kortex_driver-msg:oneof_type instead.")
  (oneof_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianTrajectoryConstraint>) ostream)
  "Serializes a message object of type '<CartesianTrajectoryConstraint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_type) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianTrajectoryConstraint>) istream)
  "Deserializes a message object of type '<CartesianTrajectoryConstraint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_type) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianTrajectoryConstraint>)))
  "Returns string type for a message object of type '<CartesianTrajectoryConstraint>"
  "kortex_driver/CartesianTrajectoryConstraint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianTrajectoryConstraint)))
  "Returns string type for a message object of type 'CartesianTrajectoryConstraint"
  "kortex_driver/CartesianTrajectoryConstraint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianTrajectoryConstraint>)))
  "Returns md5sum for a message object of type '<CartesianTrajectoryConstraint>"
  "4758c371e633c2206a71f0c6833f6d5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianTrajectoryConstraint)))
  "Returns md5sum for a message object of type 'CartesianTrajectoryConstraint"
  "4758c371e633c2206a71f0c6833f6d5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianTrajectoryConstraint>)))
  "Returns full string definition for message of type '<CartesianTrajectoryConstraint>"
  (cl:format cl:nil "~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianTrajectoryConstraint)))
  "Returns full string definition for message of type 'CartesianTrajectoryConstraint"
  (cl:format cl:nil "~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianTrajectoryConstraint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianTrajectoryConstraint>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianTrajectoryConstraint
    (cl:cons ':oneof_type (oneof_type msg))
))
