; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude KinematicTrajectoryConstraints.msg.html

(cl:defclass <KinematicTrajectoryConstraints> (roslisp-msg-protocol:ros-message)
  ((angular_velocities
    :reader angular_velocities
    :initarg :angular_velocities
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (linear_velocity
    :reader linear_velocity
    :initarg :linear_velocity
    :type cl:float
    :initform 0.0)
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass KinematicTrajectoryConstraints (<KinematicTrajectoryConstraints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KinematicTrajectoryConstraints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KinematicTrajectoryConstraints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<KinematicTrajectoryConstraints> is deprecated: use kortex_driver-msg:KinematicTrajectoryConstraints instead.")))

(cl:ensure-generic-function 'angular_velocities-val :lambda-list '(m))
(cl:defmethod angular_velocities-val ((m <KinematicTrajectoryConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:angular_velocities-val is deprecated.  Use kortex_driver-msg:angular_velocities instead.")
  (angular_velocities m))

(cl:ensure-generic-function 'linear_velocity-val :lambda-list '(m))
(cl:defmethod linear_velocity-val ((m <KinematicTrajectoryConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:linear_velocity-val is deprecated.  Use kortex_driver-msg:linear_velocity instead.")
  (linear_velocity m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <KinematicTrajectoryConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:angular_velocity-val is deprecated.  Use kortex_driver-msg:angular_velocity instead.")
  (angular_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KinematicTrajectoryConstraints>) ostream)
  "Serializes a message object of type '<KinematicTrajectoryConstraints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angular_velocities))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angular_velocities))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KinematicTrajectoryConstraints>) istream)
  "Deserializes a message object of type '<KinematicTrajectoryConstraints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angular_velocities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angular_velocities)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KinematicTrajectoryConstraints>)))
  "Returns string type for a message object of type '<KinematicTrajectoryConstraints>"
  "kortex_driver/KinematicTrajectoryConstraints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KinematicTrajectoryConstraints)))
  "Returns string type for a message object of type 'KinematicTrajectoryConstraints"
  "kortex_driver/KinematicTrajectoryConstraints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KinematicTrajectoryConstraints>)))
  "Returns md5sum for a message object of type '<KinematicTrajectoryConstraints>"
  "4f0c906b29207ad54df6adc023e55732")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KinematicTrajectoryConstraints)))
  "Returns md5sum for a message object of type 'KinematicTrajectoryConstraints"
  "4f0c906b29207ad54df6adc023e55732")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KinematicTrajectoryConstraints>)))
  "Returns full string definition for message of type '<KinematicTrajectoryConstraints>"
  (cl:format cl:nil "~%float32[] angular_velocities~%float32 linear_velocity~%float32 angular_velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KinematicTrajectoryConstraints)))
  "Returns full string definition for message of type 'KinematicTrajectoryConstraints"
  (cl:format cl:nil "~%float32[] angular_velocities~%float32 linear_velocity~%float32 angular_velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KinematicTrajectoryConstraints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angular_velocities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KinematicTrajectoryConstraints>))
  "Converts a ROS message object to a list"
  (cl:list 'KinematicTrajectoryConstraints
    (cl:cons ':angular_velocities (angular_velocities msg))
    (cl:cons ':linear_velocity (linear_velocity msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
))
