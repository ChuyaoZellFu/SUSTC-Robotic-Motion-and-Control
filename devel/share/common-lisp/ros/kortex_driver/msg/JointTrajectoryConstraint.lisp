; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointTrajectoryConstraint.msg.html

(cl:defclass <JointTrajectoryConstraint> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass JointTrajectoryConstraint (<JointTrajectoryConstraint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTrajectoryConstraint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTrajectoryConstraint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointTrajectoryConstraint> is deprecated: use kortex_driver-msg:JointTrajectoryConstraint instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <JointTrajectoryConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <JointTrajectoryConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTrajectoryConstraint>) ostream)
  "Serializes a message object of type '<JointTrajectoryConstraint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTrajectoryConstraint>) istream)
  "Deserializes a message object of type '<JointTrajectoryConstraint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTrajectoryConstraint>)))
  "Returns string type for a message object of type '<JointTrajectoryConstraint>"
  "kortex_driver/JointTrajectoryConstraint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTrajectoryConstraint)))
  "Returns string type for a message object of type 'JointTrajectoryConstraint"
  "kortex_driver/JointTrajectoryConstraint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTrajectoryConstraint>)))
  "Returns md5sum for a message object of type '<JointTrajectoryConstraint>"
  "db87d33773847285bf5558a40e0eb92c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTrajectoryConstraint)))
  "Returns md5sum for a message object of type 'JointTrajectoryConstraint"
  "db87d33773847285bf5558a40e0eb92c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTrajectoryConstraint>)))
  "Returns full string definition for message of type '<JointTrajectoryConstraint>"
  (cl:format cl:nil "~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTrajectoryConstraint)))
  "Returns full string definition for message of type 'JointTrajectoryConstraint"
  (cl:format cl:nil "~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTrajectoryConstraint>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTrajectoryConstraint>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTrajectoryConstraint
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
