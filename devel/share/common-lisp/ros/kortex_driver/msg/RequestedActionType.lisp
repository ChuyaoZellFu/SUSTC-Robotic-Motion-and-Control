; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RequestedActionType.msg.html

(cl:defclass <RequestedActionType> (roslisp-msg-protocol:ros-message)
  ((action_type
    :reader action_type
    :initarg :action_type
    :type cl:integer
    :initform 0))
)

(cl:defclass RequestedActionType (<RequestedActionType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestedActionType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestedActionType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RequestedActionType> is deprecated: use kortex_driver-msg:RequestedActionType instead.")))

(cl:ensure-generic-function 'action_type-val :lambda-list '(m))
(cl:defmethod action_type-val ((m <RequestedActionType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:action_type-val is deprecated.  Use kortex_driver-msg:action_type instead.")
  (action_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestedActionType>) ostream)
  "Serializes a message object of type '<RequestedActionType>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestedActionType>) istream)
  "Deserializes a message object of type '<RequestedActionType>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestedActionType>)))
  "Returns string type for a message object of type '<RequestedActionType>"
  "kortex_driver/RequestedActionType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestedActionType)))
  "Returns string type for a message object of type 'RequestedActionType"
  "kortex_driver/RequestedActionType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestedActionType>)))
  "Returns md5sum for a message object of type '<RequestedActionType>"
  "38655324c3c40f2a5f27841a581c389b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestedActionType)))
  "Returns md5sum for a message object of type 'RequestedActionType"
  "38655324c3c40f2a5f27841a581c389b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestedActionType>)))
  "Returns full string definition for message of type '<RequestedActionType>"
  (cl:format cl:nil "~%uint32 action_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestedActionType)))
  "Returns full string definition for message of type 'RequestedActionType"
  (cl:format cl:nil "~%uint32 action_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestedActionType>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestedActionType>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestedActionType
    (cl:cons ':action_type (action_type msg))
))
