; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NetworkHandle.msg.html

(cl:defclass <NetworkHandle> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass NetworkHandle (<NetworkHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NetworkHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NetworkHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NetworkHandle> is deprecated: use kortex_driver-msg:NetworkHandle instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <NetworkHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NetworkHandle>) ostream)
  "Serializes a message object of type '<NetworkHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NetworkHandle>) istream)
  "Deserializes a message object of type '<NetworkHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NetworkHandle>)))
  "Returns string type for a message object of type '<NetworkHandle>"
  "kortex_driver/NetworkHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NetworkHandle)))
  "Returns string type for a message object of type 'NetworkHandle"
  "kortex_driver/NetworkHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NetworkHandle>)))
  "Returns md5sum for a message object of type '<NetworkHandle>"
  "fddb7a9f6752fdb043992b8a34032ae9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NetworkHandle)))
  "Returns md5sum for a message object of type 'NetworkHandle"
  "fddb7a9f6752fdb043992b8a34032ae9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NetworkHandle>)))
  "Returns full string definition for message of type '<NetworkHandle>"
  (cl:format cl:nil "~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NetworkHandle)))
  "Returns full string definition for message of type 'NetworkHandle"
  (cl:format cl:nil "~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NetworkHandle>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NetworkHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'NetworkHandle
    (cl:cons ':type (type msg))
))
