; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Timeout.msg.html

(cl:defclass <Timeout> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass Timeout (<Timeout>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Timeout>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Timeout)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Timeout> is deprecated: use kortex_driver-msg:Timeout instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Timeout>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Timeout>) ostream)
  "Serializes a message object of type '<Timeout>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Timeout>) istream)
  "Deserializes a message object of type '<Timeout>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Timeout>)))
  "Returns string type for a message object of type '<Timeout>"
  "kortex_driver/Timeout")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Timeout)))
  "Returns string type for a message object of type 'Timeout"
  "kortex_driver/Timeout")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Timeout>)))
  "Returns md5sum for a message object of type '<Timeout>"
  "d9d213e8c146065ec958333c617eb1a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Timeout)))
  "Returns md5sum for a message object of type 'Timeout"
  "d9d213e8c146065ec958333c617eb1a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Timeout>)))
  "Returns full string definition for message of type '<Timeout>"
  (cl:format cl:nil "~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Timeout)))
  "Returns full string definition for message of type 'Timeout"
  (cl:format cl:nil "~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Timeout>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Timeout>))
  "Converts a ROS message object to a list"
  (cl:list 'Timeout
    (cl:cons ':value (value msg))
))
