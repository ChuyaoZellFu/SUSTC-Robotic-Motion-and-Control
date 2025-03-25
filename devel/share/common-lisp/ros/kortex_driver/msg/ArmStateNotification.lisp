; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ArmStateNotification.msg.html

(cl:defclass <ArmStateNotification> (roslisp-msg-protocol:ros-message)
  ((active_state
    :reader active_state
    :initarg :active_state
    :type cl:integer
    :initform 0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (connection
    :reader connection
    :initarg :connection
    :type kortex_driver-msg:Connection
    :initform (cl:make-instance 'kortex_driver-msg:Connection)))
)

(cl:defclass ArmStateNotification (<ArmStateNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmStateNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmStateNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ArmStateNotification> is deprecated: use kortex_driver-msg:ArmStateNotification instead.")))

(cl:ensure-generic-function 'active_state-val :lambda-list '(m))
(cl:defmethod active_state-val ((m <ArmStateNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_state-val is deprecated.  Use kortex_driver-msg:active_state instead.")
  (active_state m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ArmStateNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ArmStateNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmStateNotification>) ostream)
  "Serializes a message object of type '<ArmStateNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmStateNotification>) istream)
  "Deserializes a message object of type '<ArmStateNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmStateNotification>)))
  "Returns string type for a message object of type '<ArmStateNotification>"
  "kortex_driver/ArmStateNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmStateNotification)))
  "Returns string type for a message object of type 'ArmStateNotification"
  "kortex_driver/ArmStateNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmStateNotification>)))
  "Returns md5sum for a message object of type '<ArmStateNotification>"
  "73ee66f2cc8dfa1e59b9413c6d13ee3a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmStateNotification)))
  "Returns md5sum for a message object of type 'ArmStateNotification"
  "73ee66f2cc8dfa1e59b9413c6d13ee3a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmStateNotification>)))
  "Returns full string definition for message of type '<ArmStateNotification>"
  (cl:format cl:nil "~%uint32 active_state~%Timestamp timestamp~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmStateNotification)))
  "Returns full string definition for message of type 'ArmStateNotification"
  (cl:format cl:nil "~%uint32 active_state~%Timestamp timestamp~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmStateNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmStateNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmStateNotification
    (cl:cons ':active_state (active_state msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':connection (connection msg))
))
