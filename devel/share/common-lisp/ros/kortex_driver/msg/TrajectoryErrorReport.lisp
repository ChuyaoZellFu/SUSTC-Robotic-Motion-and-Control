; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryErrorReport.msg.html

(cl:defclass <TrajectoryErrorReport> (roslisp-msg-protocol:ros-message)
  ((trajectory_error_elements
    :reader trajectory_error_elements
    :initarg :trajectory_error_elements
    :type (cl:vector kortex_driver-msg:TrajectoryErrorElement)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:TrajectoryErrorElement :initial-element (cl:make-instance 'kortex_driver-msg:TrajectoryErrorElement))))
)

(cl:defclass TrajectoryErrorReport (<TrajectoryErrorReport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryErrorReport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryErrorReport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryErrorReport> is deprecated: use kortex_driver-msg:TrajectoryErrorReport instead.")))

(cl:ensure-generic-function 'trajectory_error_elements-val :lambda-list '(m))
(cl:defmethod trajectory_error_elements-val ((m <TrajectoryErrorReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:trajectory_error_elements-val is deprecated.  Use kortex_driver-msg:trajectory_error_elements instead.")
  (trajectory_error_elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryErrorReport>) ostream)
  "Serializes a message object of type '<TrajectoryErrorReport>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_error_elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory_error_elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryErrorReport>) istream)
  "Deserializes a message object of type '<TrajectoryErrorReport>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_error_elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_error_elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:TrajectoryErrorElement))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryErrorReport>)))
  "Returns string type for a message object of type '<TrajectoryErrorReport>"
  "kortex_driver/TrajectoryErrorReport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryErrorReport)))
  "Returns string type for a message object of type 'TrajectoryErrorReport"
  "kortex_driver/TrajectoryErrorReport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryErrorReport>)))
  "Returns md5sum for a message object of type '<TrajectoryErrorReport>"
  "a94c619ffdcf2a4bccd610a99b960b54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryErrorReport)))
  "Returns md5sum for a message object of type 'TrajectoryErrorReport"
  "a94c619ffdcf2a4bccd610a99b960b54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryErrorReport>)))
  "Returns full string definition for message of type '<TrajectoryErrorReport>"
  (cl:format cl:nil "~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryErrorReport)))
  "Returns full string definition for message of type 'TrajectoryErrorReport"
  (cl:format cl:nil "~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryErrorReport>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_error_elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryErrorReport>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryErrorReport
    (cl:cons ':trajectory_error_elements (trajectory_error_elements msg))
))
