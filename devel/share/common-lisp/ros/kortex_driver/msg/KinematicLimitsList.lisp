; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude KinematicLimitsList.msg.html

(cl:defclass <KinematicLimitsList> (roslisp-msg-protocol:ros-message)
  ((kinematic_limits_list
    :reader kinematic_limits_list
    :initarg :kinematic_limits_list
    :type (cl:vector kortex_driver-msg:KinematicLimits)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:KinematicLimits :initial-element (cl:make-instance 'kortex_driver-msg:KinematicLimits))))
)

(cl:defclass KinematicLimitsList (<KinematicLimitsList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KinematicLimitsList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KinematicLimitsList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<KinematicLimitsList> is deprecated: use kortex_driver-msg:KinematicLimitsList instead.")))

(cl:ensure-generic-function 'kinematic_limits_list-val :lambda-list '(m))
(cl:defmethod kinematic_limits_list-val ((m <KinematicLimitsList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kinematic_limits_list-val is deprecated.  Use kortex_driver-msg:kinematic_limits_list instead.")
  (kinematic_limits_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KinematicLimitsList>) ostream)
  "Serializes a message object of type '<KinematicLimitsList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'kinematic_limits_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'kinematic_limits_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KinematicLimitsList>) istream)
  "Deserializes a message object of type '<KinematicLimitsList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'kinematic_limits_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'kinematic_limits_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:KinematicLimits))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KinematicLimitsList>)))
  "Returns string type for a message object of type '<KinematicLimitsList>"
  "kortex_driver/KinematicLimitsList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KinematicLimitsList)))
  "Returns string type for a message object of type 'KinematicLimitsList"
  "kortex_driver/KinematicLimitsList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KinematicLimitsList>)))
  "Returns md5sum for a message object of type '<KinematicLimitsList>"
  "85620e63d1ea4e2fbf21a5cb16cb2964")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KinematicLimitsList)))
  "Returns md5sum for a message object of type 'KinematicLimitsList"
  "85620e63d1ea4e2fbf21a5cb16cb2964")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KinematicLimitsList>)))
  "Returns full string definition for message of type '<KinematicLimitsList>"
  (cl:format cl:nil "~%KinematicLimits[] kinematic_limits_list~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KinematicLimitsList)))
  "Returns full string definition for message of type 'KinematicLimitsList"
  (cl:format cl:nil "~%KinematicLimits[] kinematic_limits_list~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KinematicLimitsList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'kinematic_limits_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KinematicLimitsList>))
  "Converts a ROS message object to a list"
  (cl:list 'KinematicLimitsList
    (cl:cons ':kinematic_limits_list (kinematic_limits_list msg))
))
