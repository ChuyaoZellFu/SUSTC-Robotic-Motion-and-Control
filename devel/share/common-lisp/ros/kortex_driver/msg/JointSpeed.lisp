; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointSpeed.msg.html

(cl:defclass <JointSpeed> (roslisp-msg-protocol:ros-message)
  ((joint_identifier
    :reader joint_identifier
    :initarg :joint_identifier
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass JointSpeed (<JointSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointSpeed> is deprecated: use kortex_driver-msg:JointSpeed instead.")))

(cl:ensure-generic-function 'joint_identifier-val :lambda-list '(m))
(cl:defmethod joint_identifier-val ((m <JointSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_identifier-val is deprecated.  Use kortex_driver-msg:joint_identifier instead.")
  (joint_identifier m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <JointSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <JointSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointSpeed>) ostream)
  "Serializes a message object of type '<JointSpeed>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointSpeed>) istream)
  "Deserializes a message object of type '<JointSpeed>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'joint_identifier)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointSpeed>)))
  "Returns string type for a message object of type '<JointSpeed>"
  "kortex_driver/JointSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointSpeed)))
  "Returns string type for a message object of type 'JointSpeed"
  "kortex_driver/JointSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointSpeed>)))
  "Returns md5sum for a message object of type '<JointSpeed>"
  "fd26fce9b45cd51d86512ba0cf80d2bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointSpeed)))
  "Returns md5sum for a message object of type 'JointSpeed"
  "fd26fce9b45cd51d86512ba0cf80d2bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointSpeed>)))
  "Returns full string definition for message of type '<JointSpeed>"
  (cl:format cl:nil "~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointSpeed)))
  "Returns full string definition for message of type 'JointSpeed"
  (cl:format cl:nil "~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointSpeed>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'JointSpeed
    (cl:cons ':joint_identifier (joint_identifier msg))
    (cl:cons ':value (value msg))
    (cl:cons ':duration (duration msg))
))
