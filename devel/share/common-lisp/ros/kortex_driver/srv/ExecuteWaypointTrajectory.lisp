; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ExecuteWaypointTrajectory-request.msg.html

(cl:defclass <ExecuteWaypointTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WaypointList
    :initform (cl:make-instance 'kortex_driver-msg:WaypointList)))
)

(cl:defclass ExecuteWaypointTrajectory-request (<ExecuteWaypointTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteWaypointTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteWaypointTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteWaypointTrajectory-request> is deprecated: use kortex_driver-srv:ExecuteWaypointTrajectory-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ExecuteWaypointTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteWaypointTrajectory-request>) ostream)
  "Serializes a message object of type '<ExecuteWaypointTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteWaypointTrajectory-request>) istream)
  "Deserializes a message object of type '<ExecuteWaypointTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteWaypointTrajectory-request>)))
  "Returns string type for a service object of type '<ExecuteWaypointTrajectory-request>"
  "kortex_driver/ExecuteWaypointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteWaypointTrajectory-request)))
  "Returns string type for a service object of type 'ExecuteWaypointTrajectory-request"
  "kortex_driver/ExecuteWaypointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteWaypointTrajectory-request>)))
  "Returns md5sum for a message object of type '<ExecuteWaypointTrajectory-request>"
  "6b19855f6dc5db92347832c0f2e37c96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteWaypointTrajectory-request)))
  "Returns md5sum for a message object of type 'ExecuteWaypointTrajectory-request"
  "6b19855f6dc5db92347832c0f2e37c96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteWaypointTrajectory-request>)))
  "Returns full string definition for message of type '<ExecuteWaypointTrajectory-request>"
  (cl:format cl:nil "WaypointList input~%~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteWaypointTrajectory-request)))
  "Returns full string definition for message of type 'ExecuteWaypointTrajectory-request"
  (cl:format cl:nil "WaypointList input~%~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteWaypointTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteWaypointTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteWaypointTrajectory-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ExecuteWaypointTrajectory-response.msg.html

(cl:defclass <ExecuteWaypointTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ExecuteWaypointTrajectory-response (<ExecuteWaypointTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteWaypointTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteWaypointTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ExecuteWaypointTrajectory-response> is deprecated: use kortex_driver-srv:ExecuteWaypointTrajectory-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ExecuteWaypointTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteWaypointTrajectory-response>) ostream)
  "Serializes a message object of type '<ExecuteWaypointTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteWaypointTrajectory-response>) istream)
  "Deserializes a message object of type '<ExecuteWaypointTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteWaypointTrajectory-response>)))
  "Returns string type for a service object of type '<ExecuteWaypointTrajectory-response>"
  "kortex_driver/ExecuteWaypointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteWaypointTrajectory-response)))
  "Returns string type for a service object of type 'ExecuteWaypointTrajectory-response"
  "kortex_driver/ExecuteWaypointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteWaypointTrajectory-response>)))
  "Returns md5sum for a message object of type '<ExecuteWaypointTrajectory-response>"
  "6b19855f6dc5db92347832c0f2e37c96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteWaypointTrajectory-response)))
  "Returns md5sum for a message object of type 'ExecuteWaypointTrajectory-response"
  "6b19855f6dc5db92347832c0f2e37c96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteWaypointTrajectory-response>)))
  "Returns full string definition for message of type '<ExecuteWaypointTrajectory-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteWaypointTrajectory-response)))
  "Returns full string definition for message of type 'ExecuteWaypointTrajectory-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteWaypointTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteWaypointTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteWaypointTrajectory-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteWaypointTrajectory)))
  'ExecuteWaypointTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteWaypointTrajectory)))
  'ExecuteWaypointTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteWaypointTrajectory)))
  "Returns string type for a service object of type '<ExecuteWaypointTrajectory>"
  "kortex_driver/ExecuteWaypointTrajectory")