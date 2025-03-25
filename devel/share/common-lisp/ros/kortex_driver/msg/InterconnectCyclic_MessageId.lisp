; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_MessageId.msg.html

(cl:defclass <InterconnectCyclic_MessageId> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass InterconnectCyclic_MessageId (<InterconnectCyclic_MessageId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_MessageId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_MessageId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_MessageId> is deprecated: use kortex_driver-msg:InterconnectCyclic_MessageId instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <InterconnectCyclic_MessageId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_MessageId>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_MessageId>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_MessageId>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_MessageId>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_MessageId>)))
  "Returns string type for a message object of type '<InterconnectCyclic_MessageId>"
  "kortex_driver/InterconnectCyclic_MessageId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_MessageId)))
  "Returns string type for a message object of type 'InterconnectCyclic_MessageId"
  "kortex_driver/InterconnectCyclic_MessageId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_MessageId>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_MessageId>"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_MessageId)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_MessageId"
  "8819d354d8745a64b42def74f5c8102a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_MessageId>)))
  "Returns full string definition for message of type '<InterconnectCyclic_MessageId>"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_MessageId)))
  "Returns full string definition for message of type 'InterconnectCyclic_MessageId"
  (cl:format cl:nil "~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_MessageId>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_MessageId>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_MessageId
    (cl:cons ':identifier (identifier msg))
))
