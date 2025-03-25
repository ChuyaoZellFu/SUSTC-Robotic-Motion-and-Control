; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActionNotificationList.msg.html

(cl:defclass <ActionNotificationList> (roslisp-msg-protocol:ros-message)
  ((notifications
    :reader notifications
    :initarg :notifications
    :type (cl:vector kortex_driver-msg:ActionNotification)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ActionNotification :initial-element (cl:make-instance 'kortex_driver-msg:ActionNotification))))
)

(cl:defclass ActionNotificationList (<ActionNotificationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionNotificationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionNotificationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActionNotificationList> is deprecated: use kortex_driver-msg:ActionNotificationList instead.")))

(cl:ensure-generic-function 'notifications-val :lambda-list '(m))
(cl:defmethod notifications-val ((m <ActionNotificationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:notifications-val is deprecated.  Use kortex_driver-msg:notifications instead.")
  (notifications m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionNotificationList>) ostream)
  "Serializes a message object of type '<ActionNotificationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'notifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'notifications))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionNotificationList>) istream)
  "Deserializes a message object of type '<ActionNotificationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'notifications) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'notifications)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ActionNotification))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionNotificationList>)))
  "Returns string type for a message object of type '<ActionNotificationList>"
  "kortex_driver/ActionNotificationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionNotificationList)))
  "Returns string type for a message object of type 'ActionNotificationList"
  "kortex_driver/ActionNotificationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionNotificationList>)))
  "Returns md5sum for a message object of type '<ActionNotificationList>"
  "352928abe7de8be0e08fbb370dcfa5cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionNotificationList)))
  "Returns md5sum for a message object of type 'ActionNotificationList"
  "352928abe7de8be0e08fbb370dcfa5cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionNotificationList>)))
  "Returns full string definition for message of type '<ActionNotificationList>"
  (cl:format cl:nil "~%ActionNotification[] notifications~%================================================================================~%MSG: kortex_driver/ActionNotification~%~%uint32 action_event~%ActionHandle handle~%Timestamp timestamp~%UserProfileHandle user_handle~%uint32 abort_details~%Connection connection~%TrajectoryInfo[] trajectory_info~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/TrajectoryInfo~%~%uint32 trajectory_info_type~%uint32 waypoint_index~%uint32 joint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionNotificationList)))
  "Returns full string definition for message of type 'ActionNotificationList"
  (cl:format cl:nil "~%ActionNotification[] notifications~%================================================================================~%MSG: kortex_driver/ActionNotification~%~%uint32 action_event~%ActionHandle handle~%Timestamp timestamp~%UserProfileHandle user_handle~%uint32 abort_details~%Connection connection~%TrajectoryInfo[] trajectory_info~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/TrajectoryInfo~%~%uint32 trajectory_info_type~%uint32 waypoint_index~%uint32 joint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionNotificationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'notifications) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionNotificationList>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionNotificationList
    (cl:cons ':notifications (notifications msg))
))
