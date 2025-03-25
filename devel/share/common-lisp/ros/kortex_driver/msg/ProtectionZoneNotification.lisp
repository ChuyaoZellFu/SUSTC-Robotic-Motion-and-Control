; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProtectionZoneNotification.msg.html

(cl:defclass <ProtectionZoneNotification> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:integer
    :initform 0)
   (handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:ProtectionZoneHandle
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneHandle))
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (user_handle
    :reader user_handle
    :initarg :user_handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (connection
    :reader connection
    :initarg :connection
    :type kortex_driver-msg:Connection
    :initform (cl:make-instance 'kortex_driver-msg:Connection)))
)

(cl:defclass ProtectionZoneNotification (<ProtectionZoneNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtectionZoneNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtectionZoneNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProtectionZoneNotification> is deprecated: use kortex_driver-msg:ProtectionZoneNotification instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <ProtectionZoneNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event-val is deprecated.  Use kortex_driver-msg:event instead.")
  (event m))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <ProtectionZoneNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ProtectionZoneNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <ProtectionZoneNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ProtectionZoneNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtectionZoneNotification>) ostream)
  "Serializes a message object of type '<ProtectionZoneNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtectionZoneNotification>) istream)
  "Deserializes a message object of type '<ProtectionZoneNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtectionZoneNotification>)))
  "Returns string type for a message object of type '<ProtectionZoneNotification>"
  "kortex_driver/ProtectionZoneNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtectionZoneNotification)))
  "Returns string type for a message object of type 'ProtectionZoneNotification"
  "kortex_driver/ProtectionZoneNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtectionZoneNotification>)))
  "Returns md5sum for a message object of type '<ProtectionZoneNotification>"
  "2ba41a2c5a55d12e33dfcd059ebe2676")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtectionZoneNotification)))
  "Returns md5sum for a message object of type 'ProtectionZoneNotification"
  "2ba41a2c5a55d12e33dfcd059ebe2676")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtectionZoneNotification>)))
  "Returns full string definition for message of type '<ProtectionZoneNotification>"
  (cl:format cl:nil "~%uint32 event~%ProtectionZoneHandle handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtectionZoneNotification)))
  "Returns full string definition for message of type 'ProtectionZoneNotification"
  (cl:format cl:nil "~%uint32 event~%ProtectionZoneHandle handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtectionZoneNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtectionZoneNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtectionZoneNotification
    (cl:cons ':event (event msg))
    (cl:cons ':handle (handle msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
))
