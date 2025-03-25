; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Sequence.msg.html

(cl:defclass <Sequence> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (application_data
    :reader application_data
    :initarg :application_data
    :type cl:string
    :initform "")
   (tasks
    :reader tasks
    :initarg :tasks
    :type (cl:vector kortex_driver-msg:SequenceTask)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:SequenceTask :initial-element (cl:make-instance 'kortex_driver-msg:SequenceTask))))
)

(cl:defclass Sequence (<Sequence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sequence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sequence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Sequence> is deprecated: use kortex_driver-msg:Sequence instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <Sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'application_data-val :lambda-list '(m))
(cl:defmethod application_data-val ((m <Sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:application_data-val is deprecated.  Use kortex_driver-msg:application_data instead.")
  (application_data m))

(cl:ensure-generic-function 'tasks-val :lambda-list '(m))
(cl:defmethod tasks-val ((m <Sequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:tasks-val is deprecated.  Use kortex_driver-msg:tasks instead.")
  (tasks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sequence>) ostream)
  "Serializes a message object of type '<Sequence>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'application_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'application_data))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tasks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sequence>) istream)
  "Deserializes a message object of type '<Sequence>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'application_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'application_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:SequenceTask))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sequence>)))
  "Returns string type for a message object of type '<Sequence>"
  "kortex_driver/Sequence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sequence)))
  "Returns string type for a message object of type 'Sequence"
  "kortex_driver/Sequence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sequence>)))
  "Returns md5sum for a message object of type '<Sequence>"
  "8de6be6e1a18b5154c30497a254d1809")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sequence)))
  "Returns md5sum for a message object of type 'Sequence"
  "8de6be6e1a18b5154c30497a254d1809")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sequence>)))
  "Returns full string definition for message of type '<Sequence>"
  (cl:format cl:nil "~%SequenceHandle handle~%string name~%string application_data~%SequenceTask[] tasks~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SequenceTask~%~%uint32 group_identifier~%Action action~%string application_data~%================================================================================~%MSG: kortex_driver/Action~%~%ActionHandle handle~%string name~%string application_data~%Action_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Action_action_parameters~%~%TwistCommand[] send_twist_command~%WrenchCommand[] send_wrench_command~%Base_JointSpeeds[] send_joint_speeds~%ConstrainedPose[] reach_pose~%ConstrainedJointAngles[] reach_joint_angles~%uint32[] toggle_admittance_mode~%Snapshot[] snapshot~%SwitchControlMapping[] switch_control_mapping~%uint32[] navigate_joints~%uint32[] navigate_mappings~%ChangeTwist[] change_twist~%ChangeJointSpeeds[] change_joint_speeds~%ChangeWrench[] change_wrench~%EmergencyStop[] apply_emergency_stop~%Faults[] clear_faults~%Delay[] delay~%ActionHandle[] execute_action~%GripperCommand[] send_gripper_command~%GpioCommand[] send_gpio_command~%Base_Stop[] stop_action~%PreComputedJointTrajectory[] play_pre_computed_trajectory~%SequenceHandle[] execute_sequence~%WaypointList[] execute_waypoint_list~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%================================================================================~%MSG: kortex_driver/SwitchControlMapping~%~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ChangeTwist~%~%float32 linear~%float32 angular~%================================================================================~%MSG: kortex_driver/ChangeJointSpeeds~%~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/ChangeWrench~%~%float32 force~%float32 torque~%================================================================================~%MSG: kortex_driver/EmergencyStop~%~%================================================================================~%MSG: kortex_driver/Faults~%~%================================================================================~%MSG: kortex_driver/Delay~%~%uint32 duration~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/GpioCommand~%~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%================================================================================~%MSG: kortex_driver/Base_Stop~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sequence)))
  "Returns full string definition for message of type 'Sequence"
  (cl:format cl:nil "~%SequenceHandle handle~%string name~%string application_data~%SequenceTask[] tasks~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/SequenceTask~%~%uint32 group_identifier~%Action action~%string application_data~%================================================================================~%MSG: kortex_driver/Action~%~%ActionHandle handle~%string name~%string application_data~%Action_action_parameters oneof_action_parameters~%================================================================================~%MSG: kortex_driver/ActionHandle~%~%uint32 identifier~%uint32 action_type~%uint32 permission~%================================================================================~%MSG: kortex_driver/Action_action_parameters~%~%TwistCommand[] send_twist_command~%WrenchCommand[] send_wrench_command~%Base_JointSpeeds[] send_joint_speeds~%ConstrainedPose[] reach_pose~%ConstrainedJointAngles[] reach_joint_angles~%uint32[] toggle_admittance_mode~%Snapshot[] snapshot~%SwitchControlMapping[] switch_control_mapping~%uint32[] navigate_joints~%uint32[] navigate_mappings~%ChangeTwist[] change_twist~%ChangeJointSpeeds[] change_joint_speeds~%ChangeWrench[] change_wrench~%EmergencyStop[] apply_emergency_stop~%Faults[] clear_faults~%Delay[] delay~%ActionHandle[] execute_action~%GripperCommand[] send_gripper_command~%GpioCommand[] send_gpio_command~%Base_Stop[] stop_action~%PreComputedJointTrajectory[] play_pre_computed_trajectory~%SequenceHandle[] execute_sequence~%WaypointList[] execute_waypoint_list~%================================================================================~%MSG: kortex_driver/TwistCommand~%~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%================================================================================~%MSG: kortex_driver/WrenchCommand~%~%uint32 reference_frame~%uint32 mode~%Wrench wrench~%uint32 duration~%================================================================================~%MSG: kortex_driver/Wrench~%~%float32 force_x~%float32 force_y~%float32 force_z~%float32 torque_x~%float32 torque_y~%float32 torque_z~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%================================================================================~%MSG: kortex_driver/ConstrainedPose~%~%Pose target_pose~%CartesianTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint~%~%CartesianTrajectoryConstraint_type oneof_type~%================================================================================~%MSG: kortex_driver/CartesianTrajectoryConstraint_type~%~%CartesianSpeed[] speed~%uint32[] duration~%================================================================================~%MSG: kortex_driver/CartesianSpeed~%~%float32 translation~%float32 orientation~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%================================================================================~%MSG: kortex_driver/Snapshot~%~%uint32 snapshot_type~%================================================================================~%MSG: kortex_driver/SwitchControlMapping~%~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ChangeTwist~%~%float32 linear~%float32 angular~%================================================================================~%MSG: kortex_driver/ChangeJointSpeeds~%~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/ChangeWrench~%~%float32 force~%float32 torque~%================================================================================~%MSG: kortex_driver/EmergencyStop~%~%================================================================================~%MSG: kortex_driver/Faults~%~%================================================================================~%MSG: kortex_driver/Delay~%~%uint32 duration~%================================================================================~%MSG: kortex_driver/GripperCommand~%~%uint32 mode~%Gripper gripper~%uint32 duration~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/GpioCommand~%~%uint32 port_identifier~%uint32 pin_identifier~%uint32 action~%uint32 period~%================================================================================~%MSG: kortex_driver/Base_Stop~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%================================================================================~%MSG: kortex_driver/WaypointList~%~%Waypoint[] waypoints~%float32 duration~%bool use_optimal_blending~%================================================================================~%MSG: kortex_driver/Waypoint~%~%string name~%Waypoint_type_of_waypoint oneof_type_of_waypoint~%================================================================================~%MSG: kortex_driver/Waypoint_type_of_waypoint~%~%AngularWaypoint[] angular_waypoint~%CartesianWaypoint[] cartesian_waypoint~%================================================================================~%MSG: kortex_driver/AngularWaypoint~%~%float32[] angles~%float32[] maximum_velocities~%float32 duration~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sequence>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'application_data))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sequence>))
  "Converts a ROS message object to a list"
  (cl:list 'Sequence
    (cl:cons ':handle (handle msg))
    (cl:cons ':name (name msg))
    (cl:cons ':application_data (application_data msg))
    (cl:cons ':tasks (tasks msg))
))
