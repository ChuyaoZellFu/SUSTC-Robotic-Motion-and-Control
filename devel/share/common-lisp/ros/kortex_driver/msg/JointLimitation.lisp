; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointLimitation.msg.html

(cl:defclass <JointLimitation> (roslisp-msg-protocol:ros-message)
  ((joint_identifier
    :reader joint_identifier
    :initarg :joint_identifier
    :type cl:integer
    :initform 0)
   (type
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

(cl:defclass JointLimitation (<JointLimitation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointLimitation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointLimitation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointLimitation> is deprecated: use kortex_driver-msg:JointLimitation instead.")))

(cl:ensure-generic-function 'joint_identifier-val :lambda-list '(m))
(cl:defmethod joint_identifier-val ((m <JointLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_identifier-val is deprecated.  Use kortex_driver-msg:joint_identifier instead.")
  (joint_identifier m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <JointLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <JointLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointLimitation>) ostream)
  "Serializes a message object of type '<JointLimitation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointLimitation>) istream)
  "Deserializes a message object of type '<JointLimitation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointLimitation>)))
  "Returns string type for a message object of type '<JointLimitation>"
  "kortex_driver/JointLimitation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointLimitation)))
  "Returns string type for a message object of type 'JointLimitation"
  "kortex_driver/JointLimitation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointLimitation>)))
  "Returns md5sum for a message object of type '<JointLimitation>"
  "c7e8a86926cdf71a09bb6314d52e9586")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointLimitation)))
  "Returns md5sum for a message object of type 'JointLimitation"
  "c7e8a86926cdf71a09bb6314d52e9586")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointLimitation>)))
  "Returns full string definition for message of type '<JointLimitation>"
  (cl:format cl:nil "~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointLimitation)))
  "Returns full string definition for message of type 'JointLimitation"
  (cl:format cl:nil "~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointLimitation>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointLimitation>))
  "Converts a ROS message object to a list"
  (cl:list 'JointLimitation
    (cl:cons ':joint_identifier (joint_identifier msg))
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
