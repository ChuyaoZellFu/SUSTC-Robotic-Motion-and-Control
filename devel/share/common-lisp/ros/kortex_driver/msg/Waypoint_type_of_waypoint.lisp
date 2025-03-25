; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Waypoint_type_of_waypoint.msg.html

(cl:defclass <Waypoint_type_of_waypoint> (roslisp-msg-protocol:ros-message)
  ((angular_waypoint
    :reader angular_waypoint
    :initarg :angular_waypoint
    :type (cl:vector kortex_driver-msg:AngularWaypoint)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:AngularWaypoint :initial-element (cl:make-instance 'kortex_driver-msg:AngularWaypoint)))
   (cartesian_waypoint
    :reader cartesian_waypoint
    :initarg :cartesian_waypoint
    :type (cl:vector kortex_driver-msg:CartesianWaypoint)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianWaypoint :initial-element (cl:make-instance 'kortex_driver-msg:CartesianWaypoint))))
)

(cl:defclass Waypoint_type_of_waypoint (<Waypoint_type_of_waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoint_type_of_waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoint_type_of_waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Waypoint_type_of_waypoint> is deprecated: use kortex_driver-msg:Waypoint_type_of_waypoint instead.")))

(cl:ensure-generic-function 'angular_waypoint-val :lambda-list '(m))
(cl:defmethod angular_waypoint-val ((m <Waypoint_type_of_waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:angular_waypoint-val is deprecated.  Use kortex_driver-msg:angular_waypoint instead.")
  (angular_waypoint m))

(cl:ensure-generic-function 'cartesian_waypoint-val :lambda-list '(m))
(cl:defmethod cartesian_waypoint-val ((m <Waypoint_type_of_waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:cartesian_waypoint-val is deprecated.  Use kortex_driver-msg:cartesian_waypoint instead.")
  (cartesian_waypoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoint_type_of_waypoint>) ostream)
  "Serializes a message object of type '<Waypoint_type_of_waypoint>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angular_waypoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'angular_waypoint))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cartesian_waypoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cartesian_waypoint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoint_type_of_waypoint>) istream)
  "Deserializes a message object of type '<Waypoint_type_of_waypoint>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angular_waypoint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angular_waypoint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:AngularWaypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cartesian_waypoint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cartesian_waypoint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianWaypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoint_type_of_waypoint>)))
  "Returns string type for a message object of type '<Waypoint_type_of_waypoint>"
  "kortex_driver/Waypoint_type_of_waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoint_type_of_waypoint)))
  "Returns string type for a message object of type 'Waypoint_type_of_waypoint"
  "kortex_driver/Waypoint_type_of_waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoint_type_of_waypoint>)))
  "Returns md5sum for a message object of type '<Waypoint_type_of_waypoint>"
  "90682b2ce9c17ef82e25a79e7e035287")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoint_type_of_waypoint)))
  "Returns md5sum for a message object of type 'Waypoint_type_of_waypoint"
  "90682b2ce9c17ef82e25a79e7e035287")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoint_type_of_waypoint>)))
  "Returns full string definition for message of type '<Waypoint_type_of_waypoint>"
  (cl:format cl:nil "~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoint_type_of_waypoint)))
  "Returns full string definition for message of type 'Waypoint_type_of_waypoint"
  (cl:format cl:nil "~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoint_type_of_waypoint>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angular_waypoint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cartesian_waypoint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoint_type_of_waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoint_type_of_waypoint
    (cl:cons ':angular_waypoint (angular_waypoint msg))
    (cl:cons ':cartesian_waypoint (cartesian_waypoint msg))
))
