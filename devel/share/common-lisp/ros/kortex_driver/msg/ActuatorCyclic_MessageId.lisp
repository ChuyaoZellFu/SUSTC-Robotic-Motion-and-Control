; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorCyclic_MessageId.msg.html

(cl:defclass <ActuatorCyclic_MessageId> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass ActuatorCyclic_MessageId (<ActuatorCyclic_MessageId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorCyclic_MessageId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorCyclic_MessageId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorCyclic_MessageId> is deprecated: use kortex_driver-msg:ActuatorCyclic_MessageId instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <ActuatorCyclic_MessageId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorCyclic_MessageId>) ostream)
  "Serializes a message object of type '<ActuatorCyclic_MessageId>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorCyclic_MessageId>) istream)
  "Deserializes a message object of type '<ActuatorCyclic_MessageId>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorCyclic_MessageId>)))
  "Returns string type for a message object of type '<ActuatorCyclic_MessageId>"
  "kortex_driver/ActuatorCyclic_MessageId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorCyclic_MessageId)))
  "Returns string type for a message object of type 'ActuatorCyclic_MessageId"
  "kortex_driver/ActuatorCyclic_MessageId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorCyclic_MessageId>)))
  "Returns md5sum for a message object of type '<ActuatorCyclic_MessageId>"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorCyclic_MessageId)))
  "Returns md5sum for a message object of type 'ActuatorCyclic_MessageId"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorCyclic_MessageId>)))
  "Returns full string definition for message of type '<ActuatorCyclic_MessageId>"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorCyclic_MessageId)))
  "Returns full string definition for message of type 'ActuatorCyclic_MessageId"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorCyclic_MessageId>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorCyclic_MessageId>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorCyclic_MessageId
    (cl:cons ':identifier (identifier msg))
))
