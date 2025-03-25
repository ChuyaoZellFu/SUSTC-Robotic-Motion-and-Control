; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WaypointValidationReport.msg.html

(cl:defclass <WaypointValidationReport> (roslisp-msg-protocol:ros-message)
  ((trajectory_error_report
    :reader trajectory_error_report
    :initarg :trajectory_error_report
    :type kortex_driver-msg:TrajectoryErrorReport
    :initform (cl:make-instance 'kortex_driver-msg:TrajectoryErrorReport))
   (optimal_waypoint_list
    :reader optimal_waypoint_list
    :initarg :optimal_waypoint_list
    :type kortex_driver-msg:WaypointList
    :initform (cl:make-instance 'kortex_driver-msg:WaypointList)))
)

(cl:defclass WaypointValidationReport (<WaypointValidationReport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointValidationReport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointValidationReport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WaypointValidationReport> is deprecated: use kortex_driver-msg:WaypointValidationReport instead.")))

(cl:ensure-generic-function 'trajectory_error_report-val :lambda-list '(m))
(cl:defmethod trajectory_error_report-val ((m <WaypointValidationReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:trajectory_error_report-val is deprecated.  Use kortex_driver-msg:trajectory_error_report instead.")
  (trajectory_error_report m))

(cl:ensure-generic-function 'optimal_waypoint_list-val :lambda-list '(m))
(cl:defmethod optimal_waypoint_list-val ((m <WaypointValidationReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:optimal_waypoint_list-val is deprecated.  Use kortex_driver-msg:optimal_waypoint_list instead.")
  (optimal_waypoint_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointValidationReport>) ostream)
  "Serializes a message object of type '<WaypointValidationReport>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory_error_report) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'optimal_waypoint_list) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointValidationReport>) istream)
  "Deserializes a message object of type '<WaypointValidationReport>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory_error_report) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'optimal_waypoint_list) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointValidationReport>)))
  "Returns string type for a message object of type '<WaypointValidationReport>"
  "kortex_driver/WaypointValidationReport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointValidationReport)))
  "Returns string type for a message object of type 'WaypointValidationReport"
  "kortex_driver/WaypointValidationReport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointValidationReport>)))
  "Returns md5sum for a message object of type '<WaypointValidationReport>"
  "a7dd565ec93d4c9c5950e62db23f8114")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointValidationReport)))
  "Returns md5sum for a message object of type 'WaypointValidationReport"
  "a7dd565ec93d4c9c5950e62db23f8114")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointValidationReport>)))
  "Returns full string definition for message of type '<WaypointValidationReport>"
  (cl:format cl:nil "~%TrajectoryErrorReport trajectory_error_report~%WaypointList optimal_waypoint_list~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointValidationReport)))
  "Returns full string definition for message of type 'WaypointValidationReport"
  (cl:format cl:nil "~%TrajectoryErrorReport trajectory_error_report~%WaypointList optimal_waypoint_list~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointValidationReport>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory_error_report))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'optimal_waypoint_list))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointValidationReport>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointValidationReport
    (cl:cons ':trajectory_error_report (trajectory_error_report msg))
    (cl:cons ':optimal_waypoint_list (optimal_waypoint_list msg))
))
