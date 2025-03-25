; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProtectionZoneInformation.msg.html

(cl:defclass <ProtectionZoneInformation> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:integer
    :initform 0))
)

(cl:defclass ProtectionZoneInformation (<ProtectionZoneInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtectionZoneInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtectionZoneInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProtectionZoneInformation> is deprecated: use kortex_driver-msg:ProtectionZoneInformation instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <ProtectionZoneInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event-val is deprecated.  Use kortex_driver-msg:event instead.")
  (event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtectionZoneInformation>) ostream)
  "Serializes a message object of type '<ProtectionZoneInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtectionZoneInformation>) istream)
  "Deserializes a message object of type '<ProtectionZoneInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtectionZoneInformation>)))
  "Returns string type for a message object of type '<ProtectionZoneInformation>"
  "kortex_driver/ProtectionZoneInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtectionZoneInformation)))
  "Returns string type for a message object of type 'ProtectionZoneInformation"
  "kortex_driver/ProtectionZoneInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtectionZoneInformation>)))
  "Returns md5sum for a message object of type '<ProtectionZoneInformation>"
  "85038978649328eee44c6513e8ebb7e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtectionZoneInformation)))
  "Returns md5sum for a message object of type 'ProtectionZoneInformation"
  "85038978649328eee44c6513e8ebb7e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtectionZoneInformation>)))
  "Returns full string definition for message of type '<ProtectionZoneInformation>"
  (cl:format cl:nil "~%uint32 event~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtectionZoneInformation)))
  "Returns full string definition for message of type 'ProtectionZoneInformation"
  (cl:format cl:nil "~%uint32 event~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtectionZoneInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtectionZoneInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtectionZoneInformation
    (cl:cons ':event (event msg))
))
