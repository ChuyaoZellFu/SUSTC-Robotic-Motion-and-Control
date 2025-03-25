; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude KinematicLimits.msg.html

(cl:defclass <KinematicLimits> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0)
   (twist_linear
    :reader twist_linear
    :initarg :twist_linear
    :type cl:float
    :initform 0.0)
   (twist_angular
    :reader twist_angular
    :initarg :twist_angular
    :type cl:float
    :initform 0.0)
   (joint_speed_limits
    :reader joint_speed_limits
    :initarg :joint_speed_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (joint_acceleration_limits
    :reader joint_acceleration_limits
    :initarg :joint_acceleration_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass KinematicLimits (<KinematicLimits>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KinematicLimits>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KinematicLimits)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<KinematicLimits> is deprecated: use kortex_driver-msg:KinematicLimits instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <KinematicLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_mode-val is deprecated.  Use kortex_driver-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'twist_linear-val :lambda-list '(m))
(cl:defmethod twist_linear-val ((m <KinematicLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:twist_linear-val is deprecated.  Use kortex_driver-msg:twist_linear instead.")
  (twist_linear m))

(cl:ensure-generic-function 'twist_angular-val :lambda-list '(m))
(cl:defmethod twist_angular-val ((m <KinematicLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:twist_angular-val is deprecated.  Use kortex_driver-msg:twist_angular instead.")
  (twist_angular m))

(cl:ensure-generic-function 'joint_speed_limits-val :lambda-list '(m))
(cl:defmethod joint_speed_limits-val ((m <KinematicLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_speed_limits-val is deprecated.  Use kortex_driver-msg:joint_speed_limits instead.")
  (joint_speed_limits m))

(cl:ensure-generic-function 'joint_acceleration_limits-val :lambda-list '(m))
(cl:defmethod joint_acceleration_limits-val ((m <KinematicLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_acceleration_limits-val is deprecated.  Use kortex_driver-msg:joint_acceleration_limits instead.")
  (joint_acceleration_limits m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KinematicLimits>) ostream)
  "Serializes a message object of type '<KinematicLimits>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'twist_linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'twist_angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_speed_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_speed_limits))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_acceleration_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_acceleration_limits))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KinematicLimits>) istream)
  "Deserializes a message object of type '<KinematicLimits>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'twist_linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'twist_angular) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_speed_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_speed_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_acceleration_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_acceleration_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KinematicLimits>)))
  "Returns string type for a message object of type '<KinematicLimits>"
  "kortex_driver/KinematicLimits")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KinematicLimits)))
  "Returns string type for a message object of type 'KinematicLimits"
  "kortex_driver/KinematicLimits")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KinematicLimits>)))
  "Returns md5sum for a message object of type '<KinematicLimits>"
  "5496cda94486ec0fe4a0697dda6200aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KinematicLimits)))
  "Returns md5sum for a message object of type 'KinematicLimits"
  "5496cda94486ec0fe4a0697dda6200aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KinematicLimits>)))
  "Returns full string definition for message of type '<KinematicLimits>"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KinematicLimits)))
  "Returns full string definition for message of type 'KinematicLimits"
  (cl:format cl:nil "~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KinematicLimits>))
  (cl:+ 0
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_speed_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_acceleration_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KinematicLimits>))
  "Converts a ROS message object to a list"
  (cl:list 'KinematicLimits
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':twist_linear (twist_linear msg))
    (cl:cons ':twist_angular (twist_angular msg))
    (cl:cons ':joint_speed_limits (joint_speed_limits msg))
    (cl:cons ':joint_acceleration_limits (joint_acceleration_limits msg))
))
