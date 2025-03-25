; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RebootRqst.msg.html

(cl:defclass <RebootRqst> (roslisp-msg-protocol:ros-message)
  ((delay
    :reader delay
    :initarg :delay
    :type cl:integer
    :initform 0))
)

(cl:defclass RebootRqst (<RebootRqst>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RebootRqst>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RebootRqst)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RebootRqst> is deprecated: use kortex_driver-msg:RebootRqst instead.")))

(cl:ensure-generic-function 'delay-val :lambda-list '(m))
(cl:defmethod delay-val ((m <RebootRqst>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:delay-val is deprecated.  Use kortex_driver-msg:delay instead.")
  (delay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RebootRqst>) ostream)
  "Serializes a message object of type '<RebootRqst>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'delay)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'delay)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'delay)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'delay)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RebootRqst>) istream)
  "Deserializes a message object of type '<RebootRqst>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'delay)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'delay)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'delay)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'delay)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RebootRqst>)))
  "Returns string type for a message object of type '<RebootRqst>"
  "kortex_driver/RebootRqst")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RebootRqst)))
  "Returns string type for a message object of type 'RebootRqst"
  "kortex_driver/RebootRqst")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RebootRqst>)))
  "Returns md5sum for a message object of type '<RebootRqst>"
  "5759c6917ead1447f445ebc594ffe2ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RebootRqst)))
  "Returns md5sum for a message object of type 'RebootRqst"
  "5759c6917ead1447f445ebc594ffe2ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RebootRqst>)))
  "Returns full string definition for message of type '<RebootRqst>"
  (cl:format cl:nil "~%uint32 delay~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RebootRqst)))
  "Returns full string definition for message of type 'RebootRqst"
  (cl:format cl:nil "~%uint32 delay~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RebootRqst>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RebootRqst>))
  "Converts a ROS message object to a list"
  (cl:list 'RebootRqst
    (cl:cons ':delay (delay msg))
))
