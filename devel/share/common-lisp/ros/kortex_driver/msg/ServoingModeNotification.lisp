; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ServoingModeNotification.msg.html

(cl:defclass <ServoingModeNotification> (roslisp-msg-protocol:ros-message)
  ((servoing_mode
    :reader servoing_mode
    :initarg :servoing_mode
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

(cl:defclass ServoingModeNotification (<ServoingModeNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoingModeNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoingModeNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ServoingModeNotification> is deprecated: use kortex_driver-msg:ServoingModeNotification instead.")))

(cl:ensure-generic-function 'servoing_mode-val :lambda-list '(m))
(cl:defmethod servoing_mode-val ((m <ServoingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:servoing_mode-val is deprecated.  Use kortex_driver-msg:servoing_mode instead.")
  (servoing_mode m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ServoingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <ServoingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ServoingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoingModeNotification>) ostream)
  "Serializes a message object of type '<ServoingModeNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoing_mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoingModeNotification>) istream)
  "Deserializes a message object of type '<ServoingModeNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoingModeNotification>)))
  "Returns string type for a message object of type '<ServoingModeNotification>"
  "kortex_driver/ServoingModeNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoingModeNotification)))
  "Returns string type for a message object of type 'ServoingModeNotification"
  "kortex_driver/ServoingModeNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoingModeNotification>)))
  "Returns md5sum for a message object of type '<ServoingModeNotification>"
  "cbaaac12490fcae846b29cc0756b4494")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoingModeNotification)))
  "Returns md5sum for a message object of type 'ServoingModeNotification"
  "cbaaac12490fcae846b29cc0756b4494")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoingModeNotification>)))
  "Returns full string definition for message of type '<ServoingModeNotification>"
  (cl:format cl:nil "~%uint32 servoing_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoingModeNotification)))
  "Returns full string definition for message of type 'ServoingModeNotification"
  (cl:format cl:nil "~%uint32 servoing_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoingModeNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoingModeNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoingModeNotification
    (cl:cons ':servoing_mode (servoing_mode msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
))
