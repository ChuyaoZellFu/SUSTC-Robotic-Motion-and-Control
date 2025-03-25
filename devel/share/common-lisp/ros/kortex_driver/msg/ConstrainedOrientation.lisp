; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConstrainedOrientation.msg.html

(cl:defclass <ConstrainedOrientation> (roslisp-msg-protocol:ros-message)
  ((target_orientation
    :reader target_orientation
    :initarg :target_orientation
    :type kortex_driver-msg:Orientation
    :initform (cl:make-instance 'kortex_driver-msg:Orientation))
   (constraint
    :reader constraint
    :initarg :constraint
    :type kortex_driver-msg:CartesianTrajectoryConstraint
    :initform (cl:make-instance 'kortex_driver-msg:CartesianTrajectoryConstraint)))
)

(cl:defclass ConstrainedOrientation (<ConstrainedOrientation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConstrainedOrientation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConstrainedOrientation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConstrainedOrientation> is deprecated: use kortex_driver-msg:ConstrainedOrientation instead.")))

(cl:ensure-generic-function 'target_orientation-val :lambda-list '(m))
(cl:defmethod target_orientation-val ((m <ConstrainedOrientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:target_orientation-val is deprecated.  Use kortex_driver-msg:target_orientation instead.")
  (target_orientation m))

(cl:ensure-generic-function 'constraint-val :lambda-list '(m))
(cl:defmethod constraint-val ((m <ConstrainedOrientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:constraint-val is deprecated.  Use kortex_driver-msg:constraint instead.")
  (constraint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConstrainedOrientation>) ostream)
  "Serializes a message object of type '<ConstrainedOrientation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_orientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConstrainedOrientation>) istream)
  "Deserializes a message object of type '<ConstrainedOrientation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_orientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConstrainedOrientation>)))
  "Returns string type for a message object of type '<ConstrainedOrientation>"
  "kortex_driver/ConstrainedOrientation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConstrainedOrientation)))
  "Returns string type for a message object of type 'ConstrainedOrientation"
  "kortex_driver/ConstrainedOrientation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConstrainedOrientation>)))
  "Returns md5sum for a message object of type '<ConstrainedOrientation>"
  "15f399eb15570e6ba491b979fdea9cfd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConstrainedOrientation)))
  "Returns md5sum for a message object of type 'ConstrainedOrientation"
  "15f399eb15570e6ba491b979fdea9cfd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConstrainedOrientation>)))
  "Returns full string definition for message of type '<ConstrainedOrientation>"
  (cl:format cl:nil "~%Orientation target_orientation~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Orientation~%~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConstrainedOrientation)))
  "Returns full string definition for message of type 'ConstrainedOrientation"
  (cl:format cl:nil "~%Orientation target_orientation~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Orientation~%~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConstrainedOrientation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_orientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConstrainedOrientation>))
  "Converts a ROS message object to a list"
  (cl:list 'ConstrainedOrientation
    (cl:cons ':target_orientation (target_orientation msg))
    (cl:cons ':constraint (constraint msg))
))
