; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ApiOptions.msg.html

(cl:defclass <ApiOptions> (roslisp-msg-protocol:ros-message)
  ((timeout_ms
    :reader timeout_ms
    :initarg :timeout_ms
    :type cl:integer
    :initform 0))
)

(cl:defclass ApiOptions (<ApiOptions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ApiOptions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ApiOptions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ApiOptions> is deprecated: use kortex_driver-msg:ApiOptions instead.")))

(cl:ensure-generic-function 'timeout_ms-val :lambda-list '(m))
(cl:defmethod timeout_ms-val ((m <ApiOptions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timeout_ms-val is deprecated.  Use kortex_driver-msg:timeout_ms instead.")
  (timeout_ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ApiOptions>) ostream)
  "Serializes a message object of type '<ApiOptions>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout_ms)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ApiOptions>) istream)
  "Deserializes a message object of type '<ApiOptions>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timeout_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timeout_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timeout_ms)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ApiOptions>)))
  "Returns string type for a message object of type '<ApiOptions>"
  "kortex_driver/ApiOptions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApiOptions)))
  "Returns string type for a message object of type 'ApiOptions"
  "kortex_driver/ApiOptions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ApiOptions>)))
  "Returns md5sum for a message object of type '<ApiOptions>"
  "88b8e888a6c1e4d612523746429635b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ApiOptions)))
  "Returns md5sum for a message object of type 'ApiOptions"
  "88b8e888a6c1e4d612523746429635b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ApiOptions>)))
  "Returns full string definition for message of type '<ApiOptions>"
  (cl:format cl:nil "uint32 timeout_ms~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ApiOptions)))
  "Returns full string definition for message of type 'ApiOptions"
  (cl:format cl:nil "uint32 timeout_ms~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ApiOptions>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ApiOptions>))
  "Converts a ROS message object to a list"
  (cl:list 'ApiOptions
    (cl:cons ':timeout_ms (timeout_ms msg))
))
