; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyNotification.msg.html

(cl:defclass <SafetyNotification> (roslisp-msg-protocol:ros-message)
  ((safety_handle
    :reader safety_handle
    :initarg :safety_handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle))
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0)
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

(cl:defclass SafetyNotification (<SafetyNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyNotification> is deprecated: use kortex_driver-msg:SafetyNotification instead.")))

(cl:ensure-generic-function 'safety_handle-val :lambda-list '(m))
(cl:defmethod safety_handle-val ((m <SafetyNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:safety_handle-val is deprecated.  Use kortex_driver-msg:safety_handle instead.")
  (safety_handle m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SafetyNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <SafetyNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <SafetyNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <SafetyNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyNotification>) ostream)
  "Serializes a message object of type '<SafetyNotification>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'safety_handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyNotification>) istream)
  "Deserializes a message object of type '<SafetyNotification>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'safety_handle) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyNotification>)))
  "Returns string type for a message object of type '<SafetyNotification>"
  "kortex_driver/SafetyNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyNotification)))
  "Returns string type for a message object of type 'SafetyNotification"
  "kortex_driver/SafetyNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyNotification>)))
  "Returns md5sum for a message object of type '<SafetyNotification>"
  "dffadcb533e64106d087a7e7e2548f72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyNotification)))
  "Returns md5sum for a message object of type 'SafetyNotification"
  "dffadcb533e64106d087a7e7e2548f72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyNotification>)))
  "Returns full string definition for message of type '<SafetyNotification>"
  (cl:format cl:nil "~%SafetyHandle safety_handle~%uint32 value~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyNotification)))
  "Returns full string definition for message of type 'SafetyNotification"
  (cl:format cl:nil "~%SafetyHandle safety_handle~%uint32 value~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyNotification>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'safety_handle))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyNotification
    (cl:cons ':safety_handle (safety_handle msg))
    (cl:cons ':value (value msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
))
