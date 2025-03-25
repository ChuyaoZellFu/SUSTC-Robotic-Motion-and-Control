; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConfigurationChangeNotification.msg.html

(cl:defclass <ConfigurationChangeNotification> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
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
   (oneof_configuration_change
    :reader oneof_configuration_change
    :initarg :oneof_configuration_change
    :type kortex_driver-msg:ConfigurationChangeNotification_configuration_change
    :initform (cl:make-instance 'kortex_driver-msg:ConfigurationChangeNotification_configuration_change)))
)

(cl:defclass ConfigurationChangeNotification (<ConfigurationChangeNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurationChangeNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurationChangeNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConfigurationChangeNotification> is deprecated: use kortex_driver-msg:ConfigurationChangeNotification instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <ConfigurationChangeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:event-val is deprecated.  Use kortex_driver-msg:event instead.")
  (event m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ConfigurationChangeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <ConfigurationChangeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ConfigurationChangeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))

(cl:ensure-generic-function 'oneof_configuration_change-val :lambda-list '(m))
(cl:defmethod oneof_configuration_change-val ((m <ConfigurationChangeNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_configuration_change-val is deprecated.  Use kortex_driver-msg:oneof_configuration_change instead.")
  (oneof_configuration_change m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurationChangeNotification>) ostream)
  "Serializes a message object of type '<ConfigurationChangeNotification>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_configuration_change) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurationChangeNotification>) istream)
  "Deserializes a message object of type '<ConfigurationChangeNotification>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'event)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'event)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_configuration_change) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurationChangeNotification>)))
  "Returns string type for a message object of type '<ConfigurationChangeNotification>"
  "kortex_driver/ConfigurationChangeNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationChangeNotification)))
  "Returns string type for a message object of type 'ConfigurationChangeNotification"
  "kortex_driver/ConfigurationChangeNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurationChangeNotification>)))
  "Returns md5sum for a message object of type '<ConfigurationChangeNotification>"
  "3adec35b48ab707b96b990bb86718c1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurationChangeNotification)))
  "Returns md5sum for a message object of type 'ConfigurationChangeNotification"
  "3adec35b48ab707b96b990bb86718c1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurationChangeNotification>)))
  "Returns full string definition for message of type '<ConfigurationChangeNotification>"
  (cl:format cl:nil "~%uint32 event~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ConfigurationChangeNotification_configuration_change oneof_configuration_change~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification_configuration_change~%~%SequenceHandle[] sequence_handle~%ActionHandle[] action_handle~%MappingHandle[] mapping_handle~%MapGroupHandle[] map_group_handle~%MapHandle[] map_handle~%UserProfileHandle[] user_profile_handle~%ProtectionZoneHandle[] protection_zone_handle~%SafetyHandle[] safety_handle~%NetworkHandle[] network_handle~%Ssid[] ssid~%ControllerHandle[] controller_handle~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurationChangeNotification)))
  "Returns full string definition for message of type 'ConfigurationChangeNotification"
  (cl:format cl:nil "~%uint32 event~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ConfigurationChangeNotification_configuration_change oneof_configuration_change~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification_configuration_change~%~%SequenceHandle[] sequence_handle~%ActionHandle[] action_handle~%MappingHandle[] mapping_handle~%MapGroupHandle[] map_group_handle~%MapHandle[] map_handle~%UserProfileHandle[] user_profile_handle~%ProtectionZoneHandle[] protection_zone_handle~%SafetyHandle[] safety_handle~%NetworkHandle[] network_handle~%Ssid[] ssid~%ControllerHandle[] controller_handle~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurationChangeNotification>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_configuration_change))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurationChangeNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurationChangeNotification
    (cl:cons ':event (event msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
    (cl:cons ':oneof_configuration_change (oneof_configuration_change msg))
))
