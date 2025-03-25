; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConfigurationChangeNotificationList.msg.html

(cl:defclass <ConfigurationChangeNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:ConfigurationChangeNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ConfigurationChangeNotification :initial-element (cl:make-instance 'kortex_driver-msg:ConfigurationChangeNotification))))
)

(cl:defclass ConfigurationChangeNotificationList (<ConfigurationChangeNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurationChangeNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurationChangeNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConfigurationChangeNotificationList> is deprecated: use kortex_driver-msg:ConfigurationChangeNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <ConfigurationChangeNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurationChangeNotificationList>) ostream)
  "Serializes a message object of type '<ConfigurationChangeNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurationChangeNotificationList>) istream)
  "Deserializes a message object of type '<ConfigurationChangeNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ConfigurationChangeNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurationChangeNotificationList>)))
  "Returns string type for a message object of type '<ConfigurationChangeNotificationList>"
  "kortex_driver/ConfigurationChangeNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationChangeNotificationList)))
  "Returns string type for a message object of type 'ConfigurationChangeNotificationList"
  "kortex_driver/ConfigurationChangeNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurationChangeNotificationList>)))
  "Returns md5sum for a message object of type '<ConfigurationChangeNotificationList>"
  "bf8ec7af96a2de99f1b33e2bf4146af1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurationChangeNotificationList)))
  "Returns md5sum for a message object of type 'ConfigurationChangeNotificationList"
  "bf8ec7af96a2de99f1b33e2bf4146af1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurationChangeNotificationList>)))
  "Returns full string definition for message of type '<ConfigurationChangeNotificationList>"
  (cl:format cl:nil "~%ConfigurationChangeNotification[] notifications~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification~%~%uint32 event~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ConfigurationChangeNotification_configuration_change oneof_configuration_change~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification_configuration_change~%~%SequenceHandle[] sequence_handle~%ActionHandle[] action_handle~%MappingHandle[] mapping_handle~%MapGroupHandle[] map_group_handle~%MapHandle[] map_handle~%UserProfileHandle[] user_profile_handle~%ProtectionZoneHandle[] protection_zone_handle~%SafetyHandle[] safety_handle~%NetworkHandle[] network_handle~%Ssid[] ssid~%ControllerHandle[] controller_handle~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurationChangeNotificationList)))
  "Returns full string definition for message of type 'ConfigurationChangeNotificationList"
  (cl:format cl:nil "~%ConfigurationChangeNotification[] notifications~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification~%~%uint32 event~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ConfigurationChangeNotification_configuration_change oneof_configuration_change~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ConfigurationChangeNotification_configuration_change~%~%SequenceHandle[] sequence_handle~%ActionHandle[] action_handle~%MappingHandle[] mapping_handle~%MapGroupHandle[] map_group_handle~%MapHandle[] map_handle~%UserProfileHandle[] user_profile_handle~%ProtectionZoneHandle[] protection_zone_handle~%SafetyHandle[] safety_handle~%NetworkHandle[] network_handle~%Ssid[] ssid~%ControllerHandle[] controller_handle~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurationChangeNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurationChangeNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurationChangeNotificationList
    (cl:cons ':notifications (notifications msg))
))
