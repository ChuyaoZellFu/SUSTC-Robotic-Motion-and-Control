; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorInformation.msg.html

(cl:defclass <ActuatorInformation> (roslisp-msg-protocol:ros-message)
  ((count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass ActuatorInformation (<ActuatorInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorInformation> is deprecated: use kortex_driver-msg:ActuatorInformation instead.")))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <ActuatorInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:count-val is deprecated.  Use kortex_driver-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorInformation>) ostream)
  "Serializes a message object of type '<ActuatorInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorInformation>) istream)
  "Deserializes a message object of type '<ActuatorInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorInformation>)))
  "Returns string type for a message object of type '<ActuatorInformation>"
  "kortex_driver/ActuatorInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorInformation)))
  "Returns string type for a message object of type 'ActuatorInformation"
  "kortex_driver/ActuatorInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorInformation>)))
  "Returns md5sum for a message object of type '<ActuatorInformation>"
  "ac8b22eb02c1f433e0a55ee9aac59a18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorInformation)))
  "Returns md5sum for a message object of type 'ActuatorInformation"
  "ac8b22eb02c1f433e0a55ee9aac59a18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorInformation>)))
  "Returns full string definition for message of type '<ActuatorInformation>"
  (cl:format cl:nil "~%uint32 count~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorInformation)))
  "Returns full string definition for message of type 'ActuatorInformation"
  (cl:format cl:nil "~%uint32 count~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorInformation
    (cl:cons ':count (count msg))
))
