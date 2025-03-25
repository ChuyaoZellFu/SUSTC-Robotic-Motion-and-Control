; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointAccelerationSoftLimits.msg.html

(cl:defclass <JointAccelerationSoftLimits> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0)
   (joint_acceleration_soft_limits
    :reader joint_acceleration_soft_limits
    :initarg :joint_acceleration_soft_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass JointAccelerationSoftLimits (<JointAccelerationSoftLimits>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointAccelerationSoftLimits>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointAccelerationSoftLimits)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointAccelerationSoftLimits> is deprecated: use kortex_driver-msg:JointAccelerationSoftLimits instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <JointAccelerationSoftLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_mode-val is deprecated.  Use kortex_driver-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'joint_acceleration_soft_limits-val :lambda-list '(m))
(cl:defmethod joint_acceleration_soft_limits-val ((m <JointAccelerationSoftLimits>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_acceleration_soft_limits-val is deprecated.  Use kortex_driver-msg:joint_acceleration_soft_limits instead.")
  (joint_acceleration_soft_limits m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointAccelerationSoftLimits>) ostream)
  "Serializes a message object of type '<JointAccelerationSoftLimits>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_acceleration_soft_limits))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_acceleration_soft_limits))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointAccelerationSoftLimits>) istream)
  "Deserializes a message object of type '<JointAccelerationSoftLimits>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_acceleration_soft_limits) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_acceleration_soft_limits)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointAccelerationSoftLimits>)))
  "Returns string type for a message object of type '<JointAccelerationSoftLimits>"
  "kortex_driver/JointAccelerationSoftLimits")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointAccelerationSoftLimits)))
  "Returns string type for a message object of type 'JointAccelerationSoftLimits"
  "kortex_driver/JointAccelerationSoftLimits")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointAccelerationSoftLimits>)))
  "Returns md5sum for a message object of type '<JointAccelerationSoftLimits>"
  "69f7da5340a05ca3142f1af9be835bb9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointAccelerationSoftLimits)))
  "Returns md5sum for a message object of type 'JointAccelerationSoftLimits"
  "69f7da5340a05ca3142f1af9be835bb9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointAccelerationSoftLimits>)))
  "Returns full string definition for message of type '<JointAccelerationSoftLimits>"
  (cl:format cl:nil "~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointAccelerationSoftLimits)))
  "Returns full string definition for message of type 'JointAccelerationSoftLimits"
  (cl:format cl:nil "~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointAccelerationSoftLimits>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_acceleration_soft_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointAccelerationSoftLimits>))
  "Converts a ROS message object to a list"
  (cl:list 'JointAccelerationSoftLimits
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':joint_acceleration_soft_limits (joint_acceleration_soft_limits msg))
))
