; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PreComputedJointTrajectory.msg.html

(cl:defclass <PreComputedJointTrajectory> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (trajectory_elements
    :reader trajectory_elements
    :initarg :trajectory_elements
    :type (cl:vector kortex_driver-msg:PreComputedJointTrajectoryElement)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:PreComputedJointTrajectoryElement :initial-element (cl:make-instance 'kortex_driver-msg:PreComputedJointTrajectoryElement))))
)

(cl:defclass PreComputedJointTrajectory (<PreComputedJointTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PreComputedJointTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PreComputedJointTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PreComputedJointTrajectory> is deprecated: use kortex_driver-msg:PreComputedJointTrajectory instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <PreComputedJointTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'trajectory_elements-val :lambda-list '(m))
(cl:defmethod trajectory_elements-val ((m <PreComputedJointTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:trajectory_elements-val is deprecated.  Use kortex_driver-msg:trajectory_elements instead.")
  (trajectory_elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PreComputedJointTrajectory>) ostream)
  "Serializes a message object of type '<PreComputedJointTrajectory>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory_elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PreComputedJointTrajectory>) istream)
  "Deserializes a message object of type '<PreComputedJointTrajectory>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:PreComputedJointTrajectoryElement))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PreComputedJointTrajectory>)))
  "Returns string type for a message object of type '<PreComputedJointTrajectory>"
  "kortex_driver/PreComputedJointTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PreComputedJointTrajectory)))
  "Returns string type for a message object of type 'PreComputedJointTrajectory"
  "kortex_driver/PreComputedJointTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PreComputedJointTrajectory>)))
  "Returns md5sum for a message object of type '<PreComputedJointTrajectory>"
  "0b9aae85f07814f6f429b1dde77280f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PreComputedJointTrajectory)))
  "Returns md5sum for a message object of type 'PreComputedJointTrajectory"
  "0b9aae85f07814f6f429b1dde77280f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PreComputedJointTrajectory>)))
  "Returns full string definition for message of type '<PreComputedJointTrajectory>"
  (cl:format cl:nil "~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PreComputedJointTrajectory)))
  "Returns full string definition for message of type 'PreComputedJointTrajectory"
  (cl:format cl:nil "~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PreComputedJointTrajectory>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PreComputedJointTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'PreComputedJointTrajectory
    (cl:cons ':mode (mode msg))
    (cl:cons ':trajectory_elements (trajectory_elements msg))
))
