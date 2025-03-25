;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Action)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Action (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTION")
  (make-package "KORTEX_DRIVER::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass kortex_driver::Action
  :super ros::object
  :slots (_handle _name _application_data _oneof_action_parameters ))

(defmethod kortex_driver::Action
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::ActionHandle :init))
    ((:name __name) "")
    ((:application_data __application_data) "")
    ((:oneof_action_parameters __oneof_action_parameters) (instance kortex_driver::Action_action_parameters :init))
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _name (string __name))
   (setq _application_data (string __application_data))
   (setq _oneof_action_parameters __oneof_action_parameters)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:application_data
   (&optional __application_data)
   (if __application_data (setq _application_data __application_data)) _application_data)
  (:oneof_action_parameters
   (&rest __oneof_action_parameters)
   (if (keywordp (car __oneof_action_parameters))
       (send* _oneof_action_parameters __oneof_action_parameters)
     (progn
       (if __oneof_action_parameters (setq _oneof_action_parameters (car __oneof_action_parameters)))
       _oneof_action_parameters)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ActionHandle _handle
    (send _handle :serialization-length)
    ;; string _name
    4 (length _name)
    ;; string _application_data
    4 (length _application_data)
    ;; kortex_driver/Action_action_parameters _oneof_action_parameters
    (send _oneof_action_parameters :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActionHandle _handle
       (send _handle :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _application_data
       (write-long (length _application_data) s) (princ _application_data s)
     ;; kortex_driver/Action_action_parameters _oneof_action_parameters
       (send _oneof_action_parameters :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActionHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _application_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _application_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/Action_action_parameters _oneof_action_parameters
     (send _oneof_action_parameters :deserialize buf ptr-) (incf ptr- (send _oneof_action_parameters :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::Action :md5sum-) "307e297f7493c73629f241d6fa0d7aeb")
(setf (get kortex_driver::Action :datatype-) "kortex_driver/Action")
(setf (get kortex_driver::Action :definition-)
      "
ActionHandle handle
string name
string application_data
Action_action_parameters oneof_action_parameters
================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
uint32 permission
================================================================================
MSG: kortex_driver/Action_action_parameters

TwistCommand[] send_twist_command
WrenchCommand[] send_wrench_command
Base_JointSpeeds[] send_joint_speeds
ConstrainedPose[] reach_pose
ConstrainedJointAngles[] reach_joint_angles
uint32[] toggle_admittance_mode
Snapshot[] snapshot
SwitchControlMapping[] switch_control_mapping
uint32[] navigate_joints
uint32[] navigate_mappings
ChangeTwist[] change_twist
ChangeJointSpeeds[] change_joint_speeds
ChangeWrench[] change_wrench
EmergencyStop[] apply_emergency_stop
Faults[] clear_faults
Delay[] delay
ActionHandle[] execute_action
GripperCommand[] send_gripper_command
GpioCommand[] send_gpio_command
Base_Stop[] stop_action
PreComputedJointTrajectory[] play_pre_computed_trajectory
SequenceHandle[] execute_sequence
WaypointList[] execute_waypoint_list
================================================================================
MSG: kortex_driver/TwistCommand

uint32 reference_frame
Twist twist
uint32 duration
================================================================================
MSG: kortex_driver/Twist

float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
float32 angular_z
================================================================================
MSG: kortex_driver/WrenchCommand

uint32 reference_frame
uint32 mode
Wrench wrench
uint32 duration
================================================================================
MSG: kortex_driver/Wrench

float32 force_x
float32 force_y
float32 force_z
float32 torque_x
float32 torque_y
float32 torque_z
================================================================================
MSG: kortex_driver/Base_JointSpeeds

JointSpeed[] joint_speeds
uint32 duration
================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
uint32 duration
================================================================================
MSG: kortex_driver/ConstrainedPose

Pose target_pose
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint

CartesianTrajectoryConstraint_type oneof_type
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint_type

CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
float32 orientation
================================================================================
MSG: kortex_driver/ConstrainedJointAngles

JointAngles joint_angles
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
float32 value
================================================================================
MSG: kortex_driver/Snapshot

uint32 snapshot_type
================================================================================
MSG: kortex_driver/SwitchControlMapping

uint32 controller_identifier
MapGroupHandle map_group_handle
MapHandle map_handle
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ChangeTwist

float32 linear
float32 angular
================================================================================
MSG: kortex_driver/ChangeJointSpeeds

Base_JointSpeeds joint_speeds
================================================================================
MSG: kortex_driver/ChangeWrench

float32 force
float32 torque
================================================================================
MSG: kortex_driver/EmergencyStop

================================================================================
MSG: kortex_driver/Faults

================================================================================
MSG: kortex_driver/Delay

uint32 duration
================================================================================
MSG: kortex_driver/GripperCommand

uint32 mode
Gripper gripper
uint32 duration
================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
float32 value
================================================================================
MSG: kortex_driver/GpioCommand

uint32 port_identifier
uint32 pin_identifier
uint32 action
uint32 period
================================================================================
MSG: kortex_driver/Base_Stop

================================================================================
MSG: kortex_driver/PreComputedJointTrajectory

uint32 mode
PreComputedJointTrajectoryElement[] trajectory_elements
================================================================================
MSG: kortex_driver/PreComputedJointTrajectoryElement

float32[] joint_angles
float32[] joint_speeds
float32[] joint_accelerations
float32 time_from_start
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/WaypointList

Waypoint[] waypoints
float32 duration
bool use_optimal_blending
================================================================================
MSG: kortex_driver/Waypoint

string name
Waypoint_type_of_waypoint oneof_type_of_waypoint
================================================================================
MSG: kortex_driver/Waypoint_type_of_waypoint

AngularWaypoint[] angular_waypoint
CartesianWaypoint[] cartesian_waypoint
================================================================================
MSG: kortex_driver/AngularWaypoint

float32[] angles
float32[] maximum_velocities
float32 duration
================================================================================
MSG: kortex_driver/CartesianWaypoint

Pose pose
uint32 reference_frame
float32 maximum_linear_velocity
float32 maximum_angular_velocity
float32 blending_radius
")



(provide :kortex_driver/Action "307e297f7493c73629f241d6fa0d7aeb")


