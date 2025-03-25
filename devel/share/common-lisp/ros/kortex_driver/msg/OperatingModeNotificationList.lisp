; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OperatingModeNotificationList.msg.html

(cl:defclass <OperatingModeNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:OperatingModeNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:OperatingModeNotification :initial-element (cl:make-instance 'kortex_driver-msg:OperatingModeNotification))))
)

(cl:defclass OperatingModeNotificationList (<OperatingModeNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperatingModeNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperatingModeNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OperatingModeNotificationList> is deprecated: use kortex_driver-msg:OperatingModeNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <OperatingModeNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperatingModeNotificationList>) ostream)
  "Serializes a message object of type '<OperatingModeNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperatingModeNotificationList>) istream)
  "Deserializes a message object of type '<OperatingModeNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:OperatingModeNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperatingModeNotificationList>)))
  "Returns string type for a message object of type '<OperatingModeNotificationList>"
  "kortex_driver/OperatingModeNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperatingModeNotificationList)))
  "Returns string type for a message object of type 'OperatingModeNotificationList"
  "kortex_driver/OperatingModeNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperatingModeNotificationList>)))
  "Returns md5sum for a message object of type '<OperatingModeNotificationList>"
  "52f234be703b9c3a9743364c4bec1c58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperatingModeNotificationList)))
  "Returns md5sum for a message object of type 'OperatingModeNotificationList"
  "52f234be703b9c3a9743364c4bec1c58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperatingModeNotificationList>)))
  "Returns full string definition for message of type '<OperatingModeNotificationList>"
  (cl:format cl:nil "~%OperatingModeNotification[] notifications~%================================================================================~%MSG: kortex_driver/OperatingModeNotification~%~%uint32 operating_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperatingModeNotificationList)))
  "Returns full string definition for message of type 'OperatingModeNotificationList"
  (cl:format cl:nil "~%OperatingModeNotification[] notifications~%================================================================================~%MSG: kortex_driver/OperatingModeNotification~%~%uint32 operating_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%DeviceHandle device_handle~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/DeviceHandle~%~%uint32 device_type~%uint32 device_identifier~%uint32 order~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperatingModeNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperatingModeNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'OperatingModeNotificationList
    (cl:cons ':notifications (notifications msg))
))
