; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FollowCartesianTrajectoryGoal.msg.html

(cl:defclass <FollowCartesianTrajectoryGoal> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type (cl:vector kortex_driver-msg:CartesianWaypoint)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CartesianWaypoint :initial-element (cl:make-instance 'kortex_driver-msg:CartesianWaypoint)))
   (goal_time_tolerance
    :reader goal_time_tolerance
    :initarg :goal_time_tolerance
    :type cl:real
    :initform 0)
   (use_optimal_blending
    :reader use_optimal_blending
    :initarg :use_optimal_blending
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FollowCartesianTrajectoryGoal (<FollowCartesianTrajectoryGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowCartesianTrajectoryGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowCartesianTrajectoryGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FollowCartesianTrajectoryGoal> is deprecated: use kortex_driver-msg:FollowCartesianTrajectoryGoal instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:trajectory-val is deprecated.  Use kortex_driver-msg:trajectory instead.")
  (trajectory m))

(cl:ensure-generic-function 'goal_time_tolerance-val :lambda-list '(m))
(cl:defmethod goal_time_tolerance-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:goal_time_tolerance-val is deprecated.  Use kortex_driver-msg:goal_time_tolerance instead.")
  (goal_time_tolerance m))

(cl:ensure-generic-function 'use_optimal_blending-val :lambda-list '(m))
(cl:defmethod use_optimal_blending-val ((m <FollowCartesianTrajectoryGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:use_optimal_blending-val is deprecated.  Use kortex_driver-msg:use_optimal_blending instead.")
  (use_optimal_blending m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowCartesianTrajectoryGoal>) ostream)
  "Serializes a message object of type '<FollowCartesianTrajectoryGoal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'goal_time_tolerance)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'goal_time_tolerance) (cl:floor (cl:slot-value msg 'goal_time_tolerance)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_optimal_blending) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowCartesianTrajectoryGoal>) istream)
  "Deserializes a message object of type '<FollowCartesianTrajectoryGoal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CartesianWaypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goal_time_tolerance) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'use_optimal_blending) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns string type for a message object of type '<FollowCartesianTrajectoryGoal>"
  "kortex_driver/FollowCartesianTrajectoryGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns string type for a message object of type 'FollowCartesianTrajectoryGoal"
  "kortex_driver/FollowCartesianTrajectoryGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns md5sum for a message object of type '<FollowCartesianTrajectoryGoal>"
  "df06af45264ea735bb204bc1057fde50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns md5sum for a message object of type 'FollowCartesianTrajectoryGoal"
  "df06af45264ea735bb204bc1057fde50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowCartesianTrajectoryGoal>)))
  "Returns full string definition for message of type '<FollowCartesianTrajectoryGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#The trajectory to follow~%CartesianWaypoint[] trajectory~%duration goal_time_tolerance~%bool use_optimal_blending~%~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowCartesianTrajectoryGoal)))
  "Returns full string definition for message of type 'FollowCartesianTrajectoryGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#The trajectory to follow~%CartesianWaypoint[] trajectory~%duration goal_time_tolerance~%bool use_optimal_blending~%~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowCartesianTrajectoryGoal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowCartesianTrajectoryGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowCartesianTrajectoryGoal
    (cl:cons ':trajectory (trajectory msg))
    (cl:cons ':goal_time_tolerance (goal_time_tolerance msg))
    (cl:cons ':use_optimal_blending (use_optimal_blending msg))
))
