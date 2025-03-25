; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperRequest.msg.html

(cl:defclass <GripperRequest> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass GripperRequest (<GripperRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperRequest> is deprecated: use kortex_driver-msg:GripperRequest instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GripperRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperRequest>) ostream)
  "Serializes a message object of type '<GripperRequest>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperRequest>) istream)
  "Deserializes a message object of type '<GripperRequest>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperRequest>)))
  "Returns string type for a message object of type '<GripperRequest>"
  "kortex_driver/GripperRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperRequest)))
  "Returns string type for a message object of type 'GripperRequest"
  "kortex_driver/GripperRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperRequest>)))
  "Returns md5sum for a message object of type '<GripperRequest>"
  "890d2fbf5b6e45074133a9b18a5f56b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperRequest)))
  "Returns md5sum for a message object of type 'GripperRequest"
  "890d2fbf5b6e45074133a9b18a5f56b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperRequest>)))
  "Returns full string definition for message of type '<GripperRequest>"
  (cl:format cl:nil "~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperRequest)))
  "Returns full string definition for message of type 'GripperRequest"
  (cl:format cl:nil "~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperRequest>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperRequest
    (cl:cons ':mode (mode msg))
))
