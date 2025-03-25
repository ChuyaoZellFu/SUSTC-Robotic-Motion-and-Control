; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyHandle.msg.html

(cl:defclass <SafetyHandle> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass SafetyHandle (<SafetyHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyHandle> is deprecated: use kortex_driver-msg:SafetyHandle instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <SafetyHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyHandle>) ostream)
  "Serializes a message object of type '<SafetyHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyHandle>) istream)
  "Deserializes a message object of type '<SafetyHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyHandle>)))
  "Returns string type for a message object of type '<SafetyHandle>"
  "kortex_driver/SafetyHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyHandle)))
  "Returns string type for a message object of type 'SafetyHandle"
  "kortex_driver/SafetyHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyHandle>)))
  "Returns md5sum for a message object of type '<SafetyHandle>"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyHandle)))
  "Returns md5sum for a message object of type 'SafetyHandle"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyHandle>)))
  "Returns full string definition for message of type '<SafetyHandle>"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyHandle)))
  "Returns full string definition for message of type 'SafetyHandle"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyHandle>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyHandle
    (cl:cons ':identifier (identifier msg))
))
