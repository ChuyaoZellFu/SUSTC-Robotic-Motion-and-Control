; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianTrajectoryConstraint_type.msg.html

(cl:defclass <CartesianTrajectoryConstraint_type> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type (cl:vector kortex_driver-msg:CartesianSpeed)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianSpeed :initial-element (cl:make-instance 'kortex_driver-msg:CartesianSpeed)))
   (duration
    :reader duration
    :initarg :duration
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass CartesianTrajectoryConstraint_type (<CartesianTrajectoryConstraint_type>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianTrajectoryConstraint_type>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianTrajectoryConstraint_type)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianTrajectoryConstraint_type> is deprecated: use kortex_driver-msg:CartesianTrajectoryConstraint_type instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <CartesianTrajectoryConstraint_type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:speed-val is deprecated.  Use kortex_driver-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <CartesianTrajectoryConstraint_type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianTrajectoryConstraint_type>) ostream)
  "Serializes a message object of type '<CartesianTrajectoryConstraint_type>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'speed))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'duration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianTrajectoryConstraint_type>) istream)
  "Deserializes a message object of type '<CartesianTrajectoryConstraint_type>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'speed) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'speed)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianSpeed))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'duration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'duration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianTrajectoryConstraint_type>)))
  "Returns string type for a message object of type '<CartesianTrajectoryConstraint_type>"
  "kortex_driver/CartesianTrajectoryConstraint_type")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianTrajectoryConstraint_type)))
  "Returns string type for a message object of type 'CartesianTrajectoryConstraint_type"
  "kortex_driver/CartesianTrajectoryConstraint_type")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianTrajectoryConstraint_type>)))
  "Returns md5sum for a message object of type '<CartesianTrajectoryConstraint_type>"
  "e14536e767e1f700827f285e4e6fd832")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianTrajectoryConstraint_type)))
  "Returns md5sum for a message object of type 'CartesianTrajectoryConstraint_type"
  "e14536e767e1f700827f285e4e6fd832")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianTrajectoryConstraint_type>)))
  "Returns full string definition for message of type '<CartesianTrajectoryConstraint_type>"
  (cl:format cl:nil "~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianTrajectoryConstraint_type)))
  "Returns full string definition for message of type 'CartesianTrajectoryConstraint_type"
  (cl:format cl:nil "~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianTrajectoryConstraint_type>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'duration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianTrajectoryConstraint_type>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianTrajectoryConstraint_type
    (cl:cons ':speed (speed msg))
    (cl:cons ':duration (duration msg))
))
