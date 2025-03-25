; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConstrainedPosition.msg.html

(cl:defclass <ConstrainedPosition> (roslisp-msg-protocol:ros-message)
  ((target_position
    :reader target_position
    :initarg :target_position
    :type kortex_driver-msg:Base_Position
    :initform (cl:make-instance 'kortex_driver-msg:Base_Position))
   (constraint
    :reader constraint
    :initarg :constraint
    :type kortex_driver-msg:CartesianTrajectoryConstraint
    :initform (cl:make-instance 'kortex_driver-msg:CartesianTrajectoryConstraint)))
)

(cl:defclass ConstrainedPosition (<ConstrainedPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConstrainedPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConstrainedPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConstrainedPosition> is deprecated: use kortex_driver-msg:ConstrainedPosition instead.")))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <ConstrainedPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:target_position-val is deprecated.  Use kortex_driver-msg:target_position instead.")
  (target_position m))

(cl:ensure-generic-function 'constraint-val :lambda-list '(m))
(cl:defmethod constraint-val ((m <ConstrainedPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:constraint-val is deprecated.  Use kortex_driver-msg:constraint instead.")
  (constraint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConstrainedPosition>) ostream)
  "Serializes a message object of type '<ConstrainedPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConstrainedPosition>) istream)
  "Deserializes a message object of type '<ConstrainedPosition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConstrainedPosition>)))
  "Returns string type for a message object of type '<ConstrainedPosition>"
  "kortex_driver/ConstrainedPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConstrainedPosition)))
  "Returns string type for a message object of type 'ConstrainedPosition"
  "kortex_driver/ConstrainedPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConstrainedPosition>)))
  "Returns md5sum for a message object of type '<ConstrainedPosition>"
  "4c5833c3b26267ce0142362e034e1ba3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConstrainedPosition)))
  "Returns md5sum for a message object of type 'ConstrainedPosition"
  "4c5833c3b26267ce0142362e034e1ba3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConstrainedPosition>)))
  "Returns full string definition for message of type '<ConstrainedPosition>"
  (cl:format cl:nil "~%Base_Position target_position~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Base_Position~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConstrainedPosition)))
  "Returns full string definition for message of type 'ConstrainedPosition"
  (cl:format cl:nil "~%Base_Position target_position~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Base_Position~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConstrainedPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConstrainedPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'ConstrainedPosition
    (cl:cons ':target_position (target_position msg))
    (cl:cons ':constraint (constraint msg))
))
