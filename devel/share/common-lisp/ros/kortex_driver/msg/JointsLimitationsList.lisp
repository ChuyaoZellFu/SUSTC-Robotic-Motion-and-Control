; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointsLimitationsList.msg.html

(cl:defclass <JointsLimitationsList> (roslisp-msg-protocol:ros-message)
  ((joints_limitations
    :reader joints_limitations
    :initarg :joints_limitations
    :type (cl:vector kortex_driver-msg:JointLimitation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:JointLimitation :initial-element (cl:make-instance 'kortex_driver-msg:JointLimitation))))
)

(cl:defclass JointsLimitationsList (<JointsLimitationsList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointsLimitationsList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointsLimitationsList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointsLimitationsList> is deprecated: use kortex_driver-msg:JointsLimitationsList instead.")))

(cl:ensure-generic-function 'joints_limitations-val :lambda-list '(m))
(cl:defmethod joints_limitations-val ((m <JointsLimitationsList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joints_limitations-val is deprecated.  Use kortex_driver-msg:joints_limitations instead.")
  (joints_limitations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointsLimitationsList>) ostream)
  "Serializes a message object of type '<JointsLimitationsList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joints_limitations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joints_limitations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointsLimitationsList>) istream)
  "Deserializes a message object of type '<JointsLimitationsList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joints_limitations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joints_limitations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:JointLimitation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointsLimitationsList>)))
  "Returns string type for a message object of type '<JointsLimitationsList>"
  "kortex_driver/JointsLimitationsList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointsLimitationsList)))
  "Returns string type for a message object of type 'JointsLimitationsList"
  "kortex_driver/JointsLimitationsList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointsLimitationsList>)))
  "Returns md5sum for a message object of type '<JointsLimitationsList>"
  "6dfe2c39e44634a2eba8f7879d518184")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointsLimitationsList)))
  "Returns md5sum for a message object of type 'JointsLimitationsList"
  "6dfe2c39e44634a2eba8f7879d518184")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointsLimitationsList>)))
  "Returns full string definition for message of type '<JointsLimitationsList>"
  (cl:format cl:nil "~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointsLimitationsList)))
  "Returns full string definition for message of type 'JointsLimitationsList"
  (cl:format cl:nil "~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointsLimitationsList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joints_limitations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointsLimitationsList>))
  "Converts a ROS message object to a list"
  (cl:list 'JointsLimitationsList
    (cl:cons ':joints_limitations (joints_limitations msg))
))
