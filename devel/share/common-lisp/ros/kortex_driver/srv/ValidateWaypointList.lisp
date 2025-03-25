; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ValidateWaypointList-request.msg.html

(cl:defclass <ValidateWaypointList-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WaypointList
    :initform (cl:make-instance 'kortex_driver-msg:WaypointList)))
)

(cl:defclass ValidateWaypointList-request (<ValidateWaypointList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidateWaypointList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidateWaypointList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ValidateWaypointList-request> is deprecated: use kortex_driver-srv:ValidateWaypointList-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ValidateWaypointList-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidateWaypointList-request>) ostream)
  "Serializes a message object of type '<ValidateWaypointList-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidateWaypointList-request>) istream)
  "Deserializes a message object of type '<ValidateWaypointList-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidateWaypointList-request>)))
  "Returns string type for a service object of type '<ValidateWaypointList-request>"
  "kortex_driver/ValidateWaypointListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateWaypointList-request)))
  "Returns string type for a service object of type 'ValidateWaypointList-request"
  "kortex_driver/ValidateWaypointListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidateWaypointList-request>)))
  "Returns md5sum for a message object of type '<ValidateWaypointList-request>"
  "0b24f0cd37f005fabc6c65bffd727f77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidateWaypointList-request)))
  "Returns md5sum for a message object of type 'ValidateWaypointList-request"
  "0b24f0cd37f005fabc6c65bffd727f77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidateWaypointList-request>)))
  "Returns full string definition for message of type '<ValidateWaypointList-request>"
  (cl:format cl:nil "WaypointList input~%~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidateWaypointList-request)))
  "Returns full string definition for message of type 'ValidateWaypointList-request"
  (cl:format cl:nil "WaypointList input~%~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidateWaypointList-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidateWaypointList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidateWaypointList-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ValidateWaypointList-response.msg.html

(cl:defclass <ValidateWaypointList-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WaypointValidationReport
    :initform (cl:make-instance 'kortex_driver-msg:WaypointValidationReport)))
)

(cl:defclass ValidateWaypointList-response (<ValidateWaypointList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidateWaypointList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidateWaypointList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ValidateWaypointList-response> is deprecated: use kortex_driver-srv:ValidateWaypointList-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ValidateWaypointList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidateWaypointList-response>) ostream)
  "Serializes a message object of type '<ValidateWaypointList-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidateWaypointList-response>) istream)
  "Deserializes a message object of type '<ValidateWaypointList-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidateWaypointList-response>)))
  "Returns string type for a service object of type '<ValidateWaypointList-response>"
  "kortex_driver/ValidateWaypointListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateWaypointList-response)))
  "Returns string type for a service object of type 'ValidateWaypointList-response"
  "kortex_driver/ValidateWaypointListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidateWaypointList-response>)))
  "Returns md5sum for a message object of type '<ValidateWaypointList-response>"
  "0b24f0cd37f005fabc6c65bffd727f77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidateWaypointList-response)))
  "Returns md5sum for a message object of type 'ValidateWaypointList-response"
  "0b24f0cd37f005fabc6c65bffd727f77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidateWaypointList-response>)))
  "Returns full string definition for message of type '<ValidateWaypointList-response>"
  (cl:format cl:nil "WaypointValidationReport output~%~%================================================================================~%MSG: kortex_driver/WaypointValidationReport~%~%TrajectoryErrorReport trajectory_error_report~%WaypointList optimal_waypoint_list~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidateWaypointList-response)))
  "Returns full string definition for message of type 'ValidateWaypointList-response"
  (cl:format cl:nil "WaypointValidationReport output~%~%================================================================================~%MSG: kortex_driver/WaypointValidationReport~%~%TrajectoryErrorReport trajectory_error_report~%WaypointList optimal_waypoint_list~%================================================================================~%MSG: kortex_driver/TrajectoryErrorReport~%~%TrajectoryErrorElement[] trajectory_error_elements~%================================================================================~%MSG: kortex_driver/TrajectoryErrorElement~%~%uint32 error_type~%uint32 error_identifier~%float32 error_value~%float32 min_value~%float32 max_value~%uint32 index~%string message~%uint32 waypoint_index~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidateWaypointList-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidateWaypointList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidateWaypointList-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ValidateWaypointList)))
  'ValidateWaypointList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ValidateWaypointList)))
  'ValidateWaypointList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateWaypointList)))
  "Returns string type for a service object of type '<ValidateWaypointList>"
  "kortex_driver/ValidateWaypointList")