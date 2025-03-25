; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UserNotification.msg.html

(cl:defclass <UserNotification> (roslisp-msg-protocol:ros-message)
  ((user_event
    :reader user_event
    :initarg :user_event
    :type cl:integer
    :initform 0)
   (modified_user
    :reader modified_user
    :initarg :modified_user
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
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

(cl:defclass UserNotification (<UserNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UserNotification> is deprecated: use kortex_driver-msg:UserNotification instead.")))

(cl:ensure-generic-function 'user_event-val :lambda-list '(m))
(cl:defmethod user_event-val ((m <UserNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_event-val is deprecated.  Use kortex_driver-msg:user_event instead.")
  (user_event m))

(cl:ensure-generic-function 'modified_user-val :lambda-list '(m))
(cl:defmethod modified_user-val ((m <UserNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:modified_user-val is deprecated.  Use kortex_driver-msg:modified_user instead.")
  (modified_user m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <UserNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <UserNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <UserNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserNotification>) ostream)
  "Serializes a message object of type '<UserNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_event)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'modified_user) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserNotification>) istream)
  "Deserializes a message object of type '<UserNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_event)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'modified_user) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserNotification>)))
  "Returns string type for a message object of type '<UserNotification>"
  "kortex_driver/UserNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserNotification)))
  "Returns string type for a message object of type 'UserNotification"
  "kortex_driver/UserNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserNotification>)))
  "Returns md5sum for a message object of type '<UserNotification>"
  "deba2e21a3d1183442bf7c264989e80c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserNotification)))
  "Returns md5sum for a message object of type 'UserNotification"
  "deba2e21a3d1183442bf7c264989e80c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserNotification>)))
  "Returns full string definition for message of type '<UserNotification>"
  (cl:format cl:nil "~%uint32 user_event~%UserProfileHandle modified_user~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserNotification)))
  "Returns full string definition for message of type 'UserNotification"
  (cl:format cl:nil "~%uint32 user_event~%UserProfileHandle modified_user~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'modified_user))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'UserNotification
    (cl:cons ':user_event (user_event msg))
    (cl:cons ':modified_user (modified_user msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
))
