; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOIdentification.msg.html

(cl:defclass <GPIOIdentification> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass GPIOIdentification (<GPIOIdentification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOIdentification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOIdentification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOIdentification> is deprecated: use kortex_driver-msg:GPIOIdentification instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <GPIOIdentification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOIdentification>) ostream)
  "Serializes a message object of type '<GPIOIdentification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOIdentification>) istream)
  "Deserializes a message object of type '<GPIOIdentification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOIdentification>)))
  "Returns string type for a message object of type '<GPIOIdentification>"
  "kortex_driver/GPIOIdentification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOIdentification)))
  "Returns string type for a message object of type 'GPIOIdentification"
  "kortex_driver/GPIOIdentification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOIdentification>)))
  "Returns md5sum for a message object of type '<GPIOIdentification>"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOIdentification)))
  "Returns md5sum for a message object of type 'GPIOIdentification"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOIdentification>)))
  "Returns full string definition for message of type '<GPIOIdentification>"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOIdentification)))
  "Returns full string definition for message of type 'GPIOIdentification"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOIdentification>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOIdentification>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOIdentification
    (cl:cons ':identifier (identifier msg))
))
