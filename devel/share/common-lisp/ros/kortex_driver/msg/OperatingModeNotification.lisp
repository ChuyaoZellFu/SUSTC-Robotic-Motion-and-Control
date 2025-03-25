; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OperatingModeNotification.msg.html

(cl:defclass <OperatingModeNotification> (roslisp-msg-protocol:ros-message)
  ((operating_mode
    :reader operating_mode
    :initarg :operating_mode
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
    :initform (cl:make-instance 'kortex_driver-msg:Connection))
   (device_handle
    :reader device_handle
    :initarg :device_handle
    :type kortex_driver-msg:DeviceHandle
    :initform (cl:make-instance 'kortex_driver-msg:DeviceHandle)))
)

(cl:defclass OperatingModeNotification (<OperatingModeNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperatingModeNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperatingModeNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OperatingModeNotification> is deprecated: use kortex_driver-msg:OperatingModeNotification instead.")))

(cl:ensure-generic-function 'operating_mode-val :lambda-list '(m))
(cl:defmethod operating_mode-val ((m <OperatingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:operating_mode-val is deprecated.  Use kortex_driver-msg:operating_mode instead.")
  (operating_mode m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <OperatingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <OperatingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <OperatingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))

(cl:ensure-generic-function 'device_handle-val :lambda-list '(m))
(cl:defmethod device_handle-val ((m <OperatingModeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:device_handle-val is deprecated.  Use kortex_driver-msg:device_handle instead.")
  (device_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperatingModeNotification>) ostream)
  "Serializes a message object of type '<OperatingModeNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'operating_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'operating_mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'device_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperatingModeNotification>) istream)
  "Deserializes a message object of type '<OperatingModeNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'operating_mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'device_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperatingModeNotification>)))
  "Returns string type for a message object of type '<OperatingModeNotification>"
  "kortex_driver/OperatingModeNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperatingModeNotification)))
  "Returns string type for a message object of type 'OperatingModeNotification"
  "kortex_driver/OperatingModeNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperatingModeNotification>)))
  "Returns md5sum for a message object of type '<OperatingModeNotification>"
  "3307c63e67ebe8d3a49769532d9dee31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperatingModeNotification)))
  "Returns md5sum for a message object of type 'OperatingModeNotification"
  "3307c63e67ebe8d3a49769532d9dee31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperatingModeNotification>)))
  "Returns full string definition for message of type '<OperatingModeNotification>"
  (cl:format cl:nil "~%uint32 operating_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperatingModeNotification)))
  "Returns full string definition for message of type 'OperatingModeNotification"
  (cl:format cl:nil "~%uint32 operating_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperatingModeNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'device_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperatingModeNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'OperatingModeNotification
    (cl:cons ':operating_mode (operating_mode msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
    (cl:cons ':device_handle (device_handle msg))
))
