; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOState.msg.html

(cl:defclass <GPIOState> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass GPIOState (<GPIOState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOState> is deprecated: use kortex_driver-msg:GPIOState instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <GPIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GPIOState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOState>) ostream)
  "Serializes a message object of type '<GPIOState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOState>) istream)
  "Deserializes a message object of type '<GPIOState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOState>)))
  "Returns string type for a message object of type '<GPIOState>"
  "kortex_driver/GPIOState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOState)))
  "Returns string type for a message object of type 'GPIOState"
  "kortex_driver/GPIOState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOState>)))
  "Returns md5sum for a message object of type '<GPIOState>"
  "8c1bfa5d5edbf20179de7fa3e9a203b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOState)))
  "Returns md5sum for a message object of type 'GPIOState"
  "8c1bfa5d5edbf20179de7fa3e9a203b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOState>)))
  "Returns full string definition for message of type '<GPIOState>"
  (cl:format cl:nil "~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOState)))
  "Returns full string definition for message of type 'GPIOState"
  (cl:format cl:nil "~%uint32 identifier~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOState>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOState>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOState
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':value (value msg))
))
