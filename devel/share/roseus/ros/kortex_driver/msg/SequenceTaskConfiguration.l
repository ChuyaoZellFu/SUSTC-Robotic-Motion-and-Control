;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceTaskConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceTaskConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCETASKCONFIGURATION")
  (make-package "KORTEX_DRIVER::SEQUENCETASKCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude SequenceTaskConfiguration.msg.html


(defclass kortex_driver::SequenceTaskConfiguration
  :super ros::object
  :slots (_sequence_task_handle _sequence_task ))

(defmethod kortex_driver::SequenceTaskConfiguration
  (:init
   (&key
    ((:sequence_task_handle __sequence_task_handle) (instance kortex_driver::SequenceTaskHandle :init))
    ((:sequence_task __sequence_task) (instance kortex_driver::SequenceTask :init))
    )
   (send-super :init)
   (setq _sequence_task_handle __sequence_task_handle)
   (setq _sequence_task __sequence_task)
   self)
  (:sequence_task_handle
   (&rest __sequence_task_handle)
   (if (keywordp (car __sequence_task_handle))
       (send* _sequence_task_handle __sequence_task_handle)
     (progn
       (if __sequence_task_handle (setq _sequence_task_handle (car __sequence_task_handle)))
       _sequence_task_handle)))
  (:sequence_task
   (&rest __sequence_task)
   (if (keywordp (car __sequence_task))
       (send* _sequence_task __sequence_task)
     (progn
       (if __sequence_task (setq _sequence_task (car __sequence_task)))
       _sequence_task)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SequenceTaskHandle _sequence_task_handle
    (send _sequence_task_handle :serialization-length)
    ;; kortex_driver/SequenceTask _sequence_task
    (send _sequence_task :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceTaskHandle _sequence_task_handle
       (send _sequence_task_handle :serialize s)
     ;; kortex_driver/SequenceTask _sequence_task
       (send _sequence_task :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceTaskHandle _sequence_task_handle
     (send _sequence_task_handle :deserialize buf ptr-) (incf ptr- (send _sequence_task_handle :serialization-length))
   ;; kortex_driver/SequenceTask _sequence_task
     (send _sequence_task :deserialize buf ptr-) (incf ptr- (send _sequence_task :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SequenceTaskConfiguration :md5sum-) "26ff41cac9f3527da78820661a1fb75f")
(setf (get kortex_driver::SequenceTaskConfiguration :datatype-) "kortex_driver/SequenceTaskConfiguration")
(setf (get kortex_driver::SequenceTaskConfiguration :definition-)
      "
SequenceTaskHandle sequence_task_handle
SequenceTask sequence_task
================================================================================
MSG: kortex_driver/SequenceTaskHandle

SequenceHandle sequence_handle
uint32 task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/SequenceTask

uint32 group_identifier
Action action
string application_data
================================================================================
MSG: kortex_driver/Action

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



(provide :kortex_driver/SequenceTaskConfiguration "26ff41cac9f3527da78820661a1fb75f")


