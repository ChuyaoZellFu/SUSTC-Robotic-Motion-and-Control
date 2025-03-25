; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlModeNotificationList.msg.html

(cl:defclass <ControlModeNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:Base_ControlModeNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:Base_ControlModeNotification :initial-element (cl:make-instance 'kortex_driver-msg:Base_ControlModeNotification))))
)

(cl:defclass ControlModeNotificationList (<ControlModeNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlModeNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlModeNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlModeNotificationList> is deprecated: use kortex_driver-msg:ControlModeNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <ControlModeNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlModeNotificationList>) ostream)
  "Serializes a message object of type '<ControlModeNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlModeNotificationList>) istream)
  "Deserializes a message object of type '<ControlModeNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:Base_ControlModeNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlModeNotificationList>)))
  "Returns string type for a message object of type '<ControlModeNotificationList>"
  "kortex_driver/ControlModeNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlModeNotificationList)))
  "Returns string type for a message object of type 'ControlModeNotificationList"
  "kortex_driver/ControlModeNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlModeNotificationList>)))
  "Returns md5sum for a message object of type '<ControlModeNotificationList>"
  "15ea06dcb7be562d13fce4a4e5f177b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlModeNotificationList)))
  "Returns md5sum for a message object of type 'ControlModeNotificationList"
  "15ea06dcb7be562d13fce4a4e5f177b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlModeNotificationList>)))
  "Returns full string definition for message of type '<ControlModeNotificationList>"
  (cl:format cl:nil "~%Base_ControlModeNotification[] notifications~%================================================================================~%MSG: kortex_driver/Base_ControlModeNotification~%~%uint32 control_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlModeNotificationList)))
  "Returns full string definition for message of type 'ControlModeNotificationList"
  (cl:format cl:nil "~%Base_ControlModeNotification[] notifications~%================================================================================~%MSG: kortex_driver/Base_ControlModeNotification~%~%uint32 control_mode~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlModeNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlModeNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlModeNotificationList
    (cl:cons ':notifications (notifications msg))
))
