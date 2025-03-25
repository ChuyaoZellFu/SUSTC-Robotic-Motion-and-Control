; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_JointSpeeds.msg.html

(cl:defclass <Base_JointSpeeds> (roslisp-msg-protocol:ros-message)
  ((joint_speeds
    :reader joint_speeds
    :initarg :joint_speeds
    :type (cl:vector kortex_driver-msg:JointSpeed)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:JointSpeed :initial-element (cl:make-instance 'kortex_driver-msg:JointSpeed)))
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass Base_JointSpeeds (<Base_JointSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_JointSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_JointSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_JointSpeeds> is deprecated: use kortex_driver-msg:Base_JointSpeeds instead.")))

(cl:ensure-generic-function 'joint_speeds-val :lambda-list '(m))
(cl:defmethod joint_speeds-val ((m <Base_JointSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_speeds-val is deprecated.  Use kortex_driver-msg:joint_speeds instead.")
  (joint_speeds m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Base_JointSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_JointSpeeds>) ostream)
  "Serializes a message object of type '<Base_JointSpeeds>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_speeds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joint_speeds))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_JointSpeeds>) istream)
  "Deserializes a message object of type '<Base_JointSpeeds>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_speeds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_speeds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:JointSpeed))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_JointSpeeds>)))
  "Returns string type for a message object of type '<Base_JointSpeeds>"
  "kortex_driver/Base_JointSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_JointSpeeds)))
  "Returns string type for a message object of type 'Base_JointSpeeds"
  "kortex_driver/Base_JointSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_JointSpeeds>)))
  "Returns md5sum for a message object of type '<Base_JointSpeeds>"
  "f323c64d027d947f420c021696580eb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_JointSpeeds)))
  "Returns md5sum for a message object of type 'Base_JointSpeeds"
  "f323c64d027d947f420c021696580eb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_JointSpeeds>)))
  "Returns full string definition for message of type '<Base_JointSpeeds>"
  (cl:format cl:nil "~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_JointSpeeds)))
  "Returns full string definition for message of type 'Base_JointSpeeds"
  (cl:format cl:nil "~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_JointSpeeds>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_speeds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_JointSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_JointSpeeds
    (cl:cons ':joint_speeds (joint_speeds msg))
    (cl:cons ':duration (duration msg))
))
