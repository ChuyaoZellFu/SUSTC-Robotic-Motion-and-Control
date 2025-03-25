; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude CreateMap-request.msg.html

(cl:defclass <CreateMap-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Map
    :initform (cl:make-instance 'kortex_driver-msg:Map)))
)

(cl:defclass CreateMap-request (<CreateMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateMap-request> is deprecated: use kortex_driver-srv:CreateMap-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <CreateMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMap-request>) ostream)
  "Serializes a message object of type '<CreateMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMap-request>) istream)
  "Deserializes a message object of type '<CreateMap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMap-request>)))
  "Returns string type for a service object of type '<CreateMap-request>"
  "kortex_driver/CreateMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMap-request)))
  "Returns string type for a service object of type 'CreateMap-request"
  "kortex_driver/CreateMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMap-request>)))
  "Returns md5sum for a message object of type '<CreateMap-request>"
  "2cd2faeb0b189ebb1e692139094333a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMap-request)))
  "Returns md5sum for a message object of type 'CreateMap-request"
  "2cd2faeb0b189ebb1e692139094333a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMap-request>)))
  "Returns full string definition for message of type '<CreateMap-request>"
  (cl:format cl:nil "Map input~%~%================================================================================~%MSG: kortex_driver/Map~%~%MapHandle handle~%string name~%MapElement[] elements~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapElement~%~%MapEvent event~%Action action~%string name~%================================================================================~%MSG: kortex_driver/MapEvent~%~%string name~%MapEvent_events oneof_events~%================================================================================~%MSG: kortex_driver/MapEvent_events~%~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/Action~%~%ActionHandle handle~%string name~%string application_data~%Action_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Action_action_parameters~%~%TwistCommand[] send_twist_command~%WrenchCommand[] send_wrench_command~%Base_JointSpeeds[] send_joint_speeds~%ConstrainedPose[] reach_pose~%ConstrainedJointAngles[] reach_joint_angles~%uint32[] toggle_admittance_mode~%Snapshot[] snapshot~%SwitchControlMapping[] switch_control_mapping~%uint32[] navigate_joints~%uint32[] navigate_mappings~%ChangeTwist[] change_twist~%ChangeJointSpeeds[] change_joint_speeds~%ChangeWrench[] change_wrench~%EmergencyStop[] apply_emergency_stop~%Faults[] clear_faults~%Delay[] delay~%ActionHandle[] execute_action~%GripperCommand[] send_gripper_command~%GpioCommand[] send_gpio_command~%Base_Stop[] stop_action~%PreComputedJointTrajectory[] play_pre_computed_trajectory~%SequenceHandle[] execute_sequence~%WaypointList[] execute_waypoint_list~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%================================================================================~%MSG: kortex_driver/SwitchControlMapping~%~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ChangeTwist~%~%float32 linear~%float32 angular~%================================================================================~%MSG: kortex_driver/ChangeJointSpeeds~%~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/ChangeWrench~%~%float32 force~%float32 torque~%================================================================================~%MSG: kortex_driver/EmergencyStop~%~%================================================================================~%MSG: kortex_driver/Faults~%~%================================================================================~%MSG: kortex_driver/Delay~%~%uint32 duration~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/GpioCommand~%~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%================================================================================~%MSG: kortex_driver/Base_Stop~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMap-request)))
  "Returns full string definition for message of type 'CreateMap-request"
  (cl:format cl:nil "Map input~%~%================================================================================~%MSG: kortex_driver/Map~%~%MapHandle handle~%string name~%MapElement[] elements~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapElement~%~%MapEvent event~%Action action~%string name~%================================================================================~%MSG: kortex_driver/MapEvent~%~%string name~%MapEvent_events oneof_events~%================================================================================~%MSG: kortex_driver/MapEvent_events~%~%SafetyEvent[] safety_event~%GpioEvent[] gpio_event~%ControllerEvent[] controller_event~%================================================================================~%MSG: kortex_driver/SafetyEvent~%~%SafetyHandle safety_handle~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/GpioEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/ControllerEvent~%~%uint32 input_type~%uint32 behavior~%uint32 input_identifier~%================================================================================~%MSG: kortex_driver/Action~%~%ActionHandle handle~%string name~%string application_data~%Action_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Action_action_parameters~%~%TwistCommand[] send_twist_command~%WrenchCommand[] send_wrench_command~%Base_JointSpeeds[] send_joint_speeds~%ConstrainedPose[] reach_pose~%ConstrainedJointAngles[] reach_joint_angles~%uint32[] toggle_admittance_mode~%Snapshot[] snapshot~%SwitchControlMapping[] switch_control_mapping~%uint32[] navigate_joints~%uint32[] navigate_mappings~%ChangeTwist[] change_twist~%ChangeJointSpeeds[] change_joint_speeds~%ChangeWrench[] change_wrench~%EmergencyStop[] apply_emergency_stop~%Faults[] clear_faults~%Delay[] delay~%ActionHandle[] execute_action~%GripperCommand[] send_gripper_command~%GpioCommand[] send_gpio_command~%Base_Stop[] stop_action~%PreComputedJointTrajectory[] play_pre_computed_trajectory~%SequenceHandle[] execute_sequence~%WaypointList[] execute_waypoint_list~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%================================================================================~%MSG: kortex_driver/SwitchControlMapping~%~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ChangeTwist~%~%float32 linear~%float32 angular~%================================================================================~%MSG: kortex_driver/ChangeJointSpeeds~%~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/ChangeWrench~%~%float32 force~%float32 torque~%================================================================================~%MSG: kortex_driver/EmergencyStop~%~%================================================================================~%MSG: kortex_driver/Faults~%~%================================================================================~%MSG: kortex_driver/Delay~%~%uint32 duration~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/GpioCommand~%~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%================================================================================~%MSG: kortex_driver/Base_Stop~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMap-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude CreateMap-response.msg.html

(cl:defclass <CreateMap-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle)))
)

(cl:defclass CreateMap-response (<CreateMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateMap-response> is deprecated: use kortex_driver-srv:CreateMap-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <CreateMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMap-response>) ostream)
  "Serializes a message object of type '<CreateMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMap-response>) istream)
  "Deserializes a message object of type '<CreateMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMap-response>)))
  "Returns string type for a service object of type '<CreateMap-response>"
  "kortex_driver/CreateMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMap-response)))
  "Returns string type for a service object of type 'CreateMap-response"
  "kortex_driver/CreateMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMap-response>)))
  "Returns md5sum for a message object of type '<CreateMap-response>"
  "2cd2faeb0b189ebb1e692139094333a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMap-response)))
  "Returns md5sum for a message object of type 'CreateMap-response"
  "2cd2faeb0b189ebb1e692139094333a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMap-response>)))
  "Returns full string definition for message of type '<CreateMap-response>"
  (cl:format cl:nil "MapHandle output~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMap-response)))
  "Returns full string definition for message of type 'CreateMap-response"
  (cl:format cl:nil "MapHandle output~%~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMap-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateMap)))
  'CreateMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateMap)))
  'CreateMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMap)))
  "Returns string type for a service object of type '<CreateMap>"
  "kortex_driver/CreateMap")