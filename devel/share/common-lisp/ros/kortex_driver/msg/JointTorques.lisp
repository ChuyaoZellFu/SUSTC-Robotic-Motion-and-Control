; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointTorques.msg.html

(cl:defclass <JointTorques> (roslisp-msg-protocol:ros-message)
  ((joint_torques
    :reader joint_torques
    :initarg :joint_torques
    :type (cl:vector kortex_driver-msg:JointTorque)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:JointTorque :initial-element (cl:make-instance 'kortex_driver-msg:JointTorque)))
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass JointTorques (<JointTorques>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTorques>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTorques)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointTorques> is deprecated: use kortex_driver-msg:JointTorques instead.")))

(cl:ensure-generic-function 'joint_torques-val :lambda-list '(m))
(cl:defmethod joint_torques-val ((m <JointTorques>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_torques-val is deprecated.  Use kortex_driver-msg:joint_torques instead.")
  (joint_torques m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <JointTorques>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTorques>) ostream)
  "Serializes a message object of type '<JointTorques>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_torques))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joint_torques))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTorques>) istream)
  "Deserializes a message object of type '<JointTorques>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_torques) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_torques)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:JointTorque))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTorques>)))
  "Returns string type for a message object of type '<JointTorques>"
  "kortex_driver/JointTorques")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTorques)))
  "Returns string type for a message object of type 'JointTorques"
  "kortex_driver/JointTorques")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTorques>)))
  "Returns md5sum for a message object of type '<JointTorques>"
  "ada0e26af44ca932f53e6c0e450fadd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTorques)))
  "Returns md5sum for a message object of type 'JointTorques"
  "ada0e26af44ca932f53e6c0e450fadd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTorques>)))
  "Returns full string definition for message of type '<JointTorques>"
  (cl:format cl:nil "~%JointTorque[] joint_torques~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointTorque~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTorques)))
  "Returns full string definition for message of type 'JointTorques"
  (cl:format cl:nil "~%JointTorque[] joint_torques~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointTorque~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTorques>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_torques) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTorques>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTorques
    (cl:cons ':joint_torques (joint_torques msg))
    (cl:cons ':duration (duration msg))
))
