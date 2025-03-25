; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MappingInfoNotification.msg.html

(cl:defclass <MappingInfoNotification> (roslisp-msg-protocol:ros-message)
  ((controller_identifier
    :reader controller_identifier
    :initarg :controller_identifier
    :type cl:integer
    :initform 0)
   (active_map_handle
    :reader active_map_handle
    :initarg :active_map_handle
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle))
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
   (mapping_handle
    :reader mapping_handle
    :initarg :mapping_handle
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle)))
)

(cl:defclass MappingInfoNotification (<MappingInfoNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MappingInfoNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MappingInfoNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MappingInfoNotification> is deprecated: use kortex_driver-msg:MappingInfoNotification instead.")))

(cl:ensure-generic-function 'controller_identifier-val :lambda-list '(m))
(cl:defmethod controller_identifier-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_identifier-val is deprecated.  Use kortex_driver-msg:controller_identifier instead.")
  (controller_identifier m))

(cl:ensure-generic-function 'active_map_handle-val :lambda-list '(m))
(cl:defmethod active_map_handle-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_map_handle-val is deprecated.  Use kortex_driver-msg:active_map_handle instead.")
  (active_map_handle m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))

(cl:ensure-generic-function 'mapping_handle-val :lambda-list '(m))
(cl:defmethod mapping_handle-val ((m <MappingInfoNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mapping_handle-val is deprecated.  Use kortex_driver-msg:mapping_handle instead.")
  (mapping_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MappingInfoNotification>) ostream)
  "Serializes a message object of type '<MappingInfoNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'active_map_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mapping_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MappingInfoNotification>) istream)
  "Deserializes a message object of type '<MappingInfoNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'active_map_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mapping_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MappingInfoNotification>)))
  "Returns string type for a message object of type '<MappingInfoNotification>"
  "kortex_driver/MappingInfoNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MappingInfoNotification)))
  "Returns string type for a message object of type 'MappingInfoNotification"
  "kortex_driver/MappingInfoNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MappingInfoNotification>)))
  "Returns md5sum for a message object of type '<MappingInfoNotification>"
  "c0d2df9a9b1143d371ae7438ebf20d09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MappingInfoNotification)))
  "Returns md5sum for a message object of type 'MappingInfoNotification"
  "c0d2df9a9b1143d371ae7438ebf20d09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MappingInfoNotification>)))
  "Returns full string definition for message of type '<MappingInfoNotification>"
  (cl:format cl:nil "~%uint32 controller_identifier~%MapHandle active_map_handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%MappingHandle mapping_handle~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MappingInfoNotification)))
  "Returns full string definition for message of type 'MappingInfoNotification"
  (cl:format cl:nil "~%uint32 controller_identifier~%MapHandle active_map_handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%MappingHandle mapping_handle~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MappingInfoNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'active_map_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mapping_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MappingInfoNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'MappingInfoNotification
    (cl:cons ':controller_identifier (controller_identifier msg))
    (cl:cons ':active_map_handle (active_map_handle msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
    (cl:cons ':mapping_handle (mapping_handle msg))
))
