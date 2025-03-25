; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerNotificationList.msg.html

(cl:defclass <ControllerNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:ControllerNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerNotification :initial-element (cl:make-instance 'kortex_driver-msg:ControllerNotification))))
)

(cl:defclass ControllerNotificationList (<ControllerNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerNotificationList> is deprecated: use kortex_driver-msg:ControllerNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <ControllerNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerNotificationList>) ostream)
  "Serializes a message object of type '<ControllerNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerNotificationList>) istream)
  "Deserializes a message object of type '<ControllerNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerNotificationList>)))
  "Returns string type for a message object of type '<ControllerNotificationList>"
  "kortex_driver/ControllerNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerNotificationList)))
  "Returns string type for a message object of type 'ControllerNotificationList"
  "kortex_driver/ControllerNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerNotificationList>)))
  "Returns md5sum for a message object of type '<ControllerNotificationList>"
  "3d9f337a9914809bd100d1e4faaea316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerNotificationList)))
  "Returns md5sum for a message object of type 'ControllerNotificationList"
  "3d9f337a9914809bd100d1e4faaea316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerNotificationList>)))
  "Returns full string definition for message of type '<ControllerNotificationList>"
  (cl:format cl:nil "~%ControllerNotification[] notifications~%================================================================================~%MSG: kortex_driver/ControllerNotification~%~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ControllerNotification_state oneof_state~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ControllerNotification_state~%~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerNotificationList)))
  "Returns full string definition for message of type 'ControllerNotificationList"
  (cl:format cl:nil "~%ControllerNotification[] notifications~%================================================================================~%MSG: kortex_driver/ControllerNotification~%~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ControllerNotification_state oneof_state~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ControllerNotification_state~%~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerNotificationList
    (cl:cons ':notifications (notifications msg))
))
