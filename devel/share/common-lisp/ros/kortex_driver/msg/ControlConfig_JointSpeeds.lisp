; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlConfig_JointSpeeds.msg.html

(cl:defclass <ControlConfig_JointSpeeds> (roslisp-msg-protocol:ros-message)
  ((joint_speed
    :reader joint_speed
    :initarg :joint_speed
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ControlConfig_JointSpeeds (<ControlConfig_JointSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_JointSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_JointSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlConfig_JointSpeeds> is deprecated: use kortex_driver-msg:ControlConfig_JointSpeeds instead.")))

(cl:ensure-generic-function 'joint_speed-val :lambda-list '(m))
(cl:defmethod joint_speed-val ((m <ControlConfig_JointSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_speed-val is deprecated.  Use kortex_driver-msg:joint_speed instead.")
  (joint_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_JointSpeeds>) ostream)
  "Serializes a message object of type '<ControlConfig_JointSpeeds>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_speed))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_JointSpeeds>) istream)
  "Deserializes a message object of type '<ControlConfig_JointSpeeds>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_speed) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_speed)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_JointSpeeds>)))
  "Returns string type for a message object of type '<ControlConfig_JointSpeeds>"
  "kortex_driver/ControlConfig_JointSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_JointSpeeds)))
  "Returns string type for a message object of type 'ControlConfig_JointSpeeds"
  "kortex_driver/ControlConfig_JointSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_JointSpeeds>)))
  "Returns md5sum for a message object of type '<ControlConfig_JointSpeeds>"
  "33fe593b138a7242a779a630fd20122e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_JointSpeeds)))
  "Returns md5sum for a message object of type 'ControlConfig_JointSpeeds"
  "33fe593b138a7242a779a630fd20122e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_JointSpeeds>)))
  "Returns full string definition for message of type '<ControlConfig_JointSpeeds>"
  (cl:format cl:nil "~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_JointSpeeds)))
  "Returns full string definition for message of type 'ControlConfig_JointSpeeds"
  (cl:format cl:nil "~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_JointSpeeds>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_JointSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_JointSpeeds
    (cl:cons ':joint_speed (joint_speed msg))
))
