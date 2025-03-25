; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MappingInfoNotificationList.msg.html

(cl:defclass <MappingInfoNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:MappingInfoNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MappingInfoNotification :initial-element (cl:make-instance 'kortex_driver-msg:MappingInfoNotification))))
)

(cl:defclass MappingInfoNotificationList (<MappingInfoNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MappingInfoNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MappingInfoNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MappingInfoNotificationList> is deprecated: use kortex_driver-msg:MappingInfoNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <MappingInfoNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MappingInfoNotificationList>) ostream)
  "Serializes a message object of type '<MappingInfoNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MappingInfoNotificationList>) istream)
  "Deserializes a message object of type '<MappingInfoNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MappingInfoNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MappingInfoNotificationList>)))
  "Returns string type for a message object of type '<MappingInfoNotificationList>"
  "kortex_driver/MappingInfoNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MappingInfoNotificationList)))
  "Returns string type for a message object of type 'MappingInfoNotificationList"
  "kortex_driver/MappingInfoNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MappingInfoNotificationList>)))
  "Returns md5sum for a message object of type '<MappingInfoNotificationList>"
  "300ba98facffe93043b4010aa2c28f88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MappingInfoNotificationList)))
  "Returns md5sum for a message object of type 'MappingInfoNotificationList"
  "300ba98facffe93043b4010aa2c28f88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MappingInfoNotificationList>)))
  "Returns full string definition for message of type '<MappingInfoNotificationList>"
  (cl:format cl:nil "~%MappingInfoNotification[] notifications~%================================================================================~%MSG: kortex_driver/MappingInfoNotification~%~%uint32 controller_identifier~%MapHandle active_map_handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%MappingHandle mapping_handle~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MappingInfoNotificationList)))
  "Returns full string definition for message of type 'MappingInfoNotificationList"
  (cl:format cl:nil "~%MappingInfoNotification[] notifications~%================================================================================~%MSG: kortex_driver/MappingInfoNotification~%~%uint32 controller_identifier~%MapHandle active_map_handle~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%MappingHandle mapping_handle~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MappingInfoNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MappingInfoNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'MappingInfoNotificationList
    (cl:cons ':notifications (notifications msg))
))
