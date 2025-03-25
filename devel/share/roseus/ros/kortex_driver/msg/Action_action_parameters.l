;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Action_action_parameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Action_action_parameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTION_ACTION_PARAMETERS")
  (make-package "KORTEX_DRIVER::ACTION_ACTION_PARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude Action_action_parameters.msg.html


(defclass kortex_driver::Action_action_parameters
  :super ros::object
  :slots (_send_twist_command _send_wrench_command _send_joint_speeds _reach_pose _reach_joint_angles _toggle_admittance_mode _snapshot _switch_control_mapping _navigate_joints _navigate_mappings _change_twist _change_joint_speeds _change_wrench _apply_emergency_stop _clear_faults _delay _execute_action _send_gripper_command _send_gpio_command _stop_action _play_pre_computed_trajectory _execute_sequence _execute_waypoint_list ))

(defmethod kortex_driver::Action_action_parameters
  (:init
   (&key
    ((:send_twist_command __send_twist_command) ())
    ((:send_wrench_command __send_wrench_command) ())
    ((:send_joint_speeds __send_joint_speeds) ())
    ((:reach_pose __reach_pose) ())
    ((:reach_joint_angles __reach_joint_angles) ())
    ((:toggle_admittance_mode __toggle_admittance_mode) (make-array 0 :initial-element 0 :element-type :integer))
    ((:snapshot __snapshot) ())
    ((:switch_control_mapping __switch_control_mapping) ())
    ((:navigate_joints __navigate_joints) (make-array 0 :initial-element 0 :element-type :integer))
    ((:navigate_mappings __navigate_mappings) (make-array 0 :initial-element 0 :element-type :integer))
    ((:change_twist __change_twist) ())
    ((:change_joint_speeds __change_joint_speeds) ())
    ((:change_wrench __change_wrench) ())
    ((:apply_emergency_stop __apply_emergency_stop) ())
    ((:clear_faults __clear_faults) ())
    ((:delay __delay) ())
    ((:execute_action __execute_action) ())
    ((:send_gripper_command __send_gripper_command) ())
    ((:send_gpio_command __send_gpio_command) ())
    ((:stop_action __stop_action) ())
    ((:play_pre_computed_trajectory __play_pre_computed_trajectory) ())
    ((:execute_sequence __execute_sequence) ())
    ((:execute_waypoint_list __execute_waypoint_list) ())
    )
   (send-super :init)
   (setq _send_twist_command __send_twist_command)
   (setq _send_wrench_command __send_wrench_command)
   (setq _send_joint_speeds __send_joint_speeds)
   (setq _reach_pose __reach_pose)
   (setq _reach_joint_angles __reach_joint_angles)
   (setq _toggle_admittance_mode __toggle_admittance_mode)
   (setq _snapshot __snapshot)
   (setq _switch_control_mapping __switch_control_mapping)
   (setq _navigate_joints __navigate_joints)
   (setq _navigate_mappings __navigate_mappings)
   (setq _change_twist __change_twist)
   (setq _change_joint_speeds __change_joint_speeds)
   (setq _change_wrench __change_wrench)
   (setq _apply_emergency_stop __apply_emergency_stop)
   (setq _clear_faults __clear_faults)
   (setq _delay __delay)
   (setq _execute_action __execute_action)
   (setq _send_gripper_command __send_gripper_command)
   (setq _send_gpio_command __send_gpio_command)
   (setq _stop_action __stop_action)
   (setq _play_pre_computed_trajectory __play_pre_computed_trajectory)
   (setq _execute_sequence __execute_sequence)
   (setq _execute_waypoint_list __execute_waypoint_list)
   self)
  (:send_twist_command
   (&rest __send_twist_command)
   (if (keywordp (car __send_twist_command))
       (send* _send_twist_command __send_twist_command)
     (progn
       (if __send_twist_command (setq _send_twist_command (car __send_twist_command)))
       _send_twist_command)))
  (:send_wrench_command
   (&rest __send_wrench_command)
   (if (keywordp (car __send_wrench_command))
       (send* _send_wrench_command __send_wrench_command)
     (progn
       (if __send_wrench_command (setq _send_wrench_command (car __send_wrench_command)))
       _send_wrench_command)))
  (:send_joint_speeds
   (&rest __send_joint_speeds)
   (if (keywordp (car __send_joint_speeds))
       (send* _send_joint_speeds __send_joint_speeds)
     (progn
       (if __send_joint_speeds (setq _send_joint_speeds (car __send_joint_speeds)))
       _send_joint_speeds)))
  (:reach_pose
   (&rest __reach_pose)
   (if (keywordp (car __reach_pose))
       (send* _reach_pose __reach_pose)
     (progn
       (if __reach_pose (setq _reach_pose (car __reach_pose)))
       _reach_pose)))
  (:reach_joint_angles
   (&rest __reach_joint_angles)
   (if (keywordp (car __reach_joint_angles))
       (send* _reach_joint_angles __reach_joint_angles)
     (progn
       (if __reach_joint_angles (setq _reach_joint_angles (car __reach_joint_angles)))
       _reach_joint_angles)))
  (:toggle_admittance_mode
   (&optional __toggle_admittance_mode)
   (if __toggle_admittance_mode (setq _toggle_admittance_mode __toggle_admittance_mode)) _toggle_admittance_mode)
  (:snapshot
   (&rest __snapshot)
   (if (keywordp (car __snapshot))
       (send* _snapshot __snapshot)
     (progn
       (if __snapshot (setq _snapshot (car __snapshot)))
       _snapshot)))
  (:switch_control_mapping
   (&rest __switch_control_mapping)
   (if (keywordp (car __switch_control_mapping))
       (send* _switch_control_mapping __switch_control_mapping)
     (progn
       (if __switch_control_mapping (setq _switch_control_mapping (car __switch_control_mapping)))
       _switch_control_mapping)))
  (:navigate_joints
   (&optional __navigate_joints)
   (if __navigate_joints (setq _navigate_joints __navigate_joints)) _navigate_joints)
  (:navigate_mappings
   (&optional __navigate_mappings)
   (if __navigate_mappings (setq _navigate_mappings __navigate_mappings)) _navigate_mappings)
  (:change_twist
   (&rest __change_twist)
   (if (keywordp (car __change_twist))
       (send* _change_twist __change_twist)
     (progn
       (if __change_twist (setq _change_twist (car __change_twist)))
       _change_twist)))
  (:change_joint_speeds
   (&rest __change_joint_speeds)
   (if (keywordp (car __change_joint_speeds))
       (send* _change_joint_speeds __change_joint_speeds)
     (progn
       (if __change_joint_speeds (setq _change_joint_speeds (car __change_joint_speeds)))
       _change_joint_speeds)))
  (:change_wrench
   (&rest __change_wrench)
   (if (keywordp (car __change_wrench))
       (send* _change_wrench __change_wrench)
     (progn
       (if __change_wrench (setq _change_wrench (car __change_wrench)))
       _change_wrench)))
  (:apply_emergency_stop
   (&rest __apply_emergency_stop)
   (if (keywordp (car __apply_emergency_stop))
       (send* _apply_emergency_stop __apply_emergency_stop)
     (progn
       (if __apply_emergency_stop (setq _apply_emergency_stop (car __apply_emergency_stop)))
       _apply_emergency_stop)))
  (:clear_faults
   (&rest __clear_faults)
   (if (keywordp (car __clear_faults))
       (send* _clear_faults __clear_faults)
     (progn
       (if __clear_faults (setq _clear_faults (car __clear_faults)))
       _clear_faults)))
  (:delay
   (&rest __delay)
   (if (keywordp (car __delay))
       (send* _delay __delay)
     (progn
       (if __delay (setq _delay (car __delay)))
       _delay)))
  (:execute_action
   (&rest __execute_action)
   (if (keywordp (car __execute_action))
       (send* _execute_action __execute_action)
     (progn
       (if __execute_action (setq _execute_action (car __execute_action)))
       _execute_action)))
  (:send_gripper_command
   (&rest __send_gripper_command)
   (if (keywordp (car __send_gripper_command))
       (send* _send_gripper_command __send_gripper_command)
     (progn
       (if __send_gripper_command (setq _send_gripper_command (car __send_gripper_command)))
       _send_gripper_command)))
  (:send_gpio_command
   (&rest __send_gpio_command)
   (if (keywordp (car __send_gpio_command))
       (send* _send_gpio_command __send_gpio_command)
     (progn
       (if __send_gpio_command (setq _send_gpio_command (car __send_gpio_command)))
       _send_gpio_command)))
  (:stop_action
   (&rest __stop_action)
   (if (keywordp (car __stop_action))
       (send* _stop_action __stop_action)
     (progn
       (if __stop_action (setq _stop_action (car __stop_action)))
       _stop_action)))
  (:play_pre_computed_trajectory
   (&rest __play_pre_computed_trajectory)
   (if (keywordp (car __play_pre_computed_trajectory))
       (send* _play_pre_computed_trajectory __play_pre_computed_trajectory)
     (progn
       (if __play_pre_computed_trajectory (setq _play_pre_computed_trajectory (car __play_pre_computed_trajectory)))
       _play_pre_computed_trajectory)))
  (:execute_sequence
   (&rest __execute_sequence)
   (if (keywordp (car __execute_sequence))
       (send* _execute_sequence __execute_sequence)
     (progn
       (if __execute_sequence (setq _execute_sequence (car __execute_sequence)))
       _execute_sequence)))
  (:execute_waypoint_list
   (&rest __execute_waypoint_list)
   (if (keywordp (car __execute_waypoint_list))
       (send* _execute_waypoint_list __execute_waypoint_list)
     (progn
       (if __execute_waypoint_list (setq _execute_waypoint_list (car __execute_waypoint_list)))
       _execute_waypoint_list)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/TwistCommand[] _send_twist_command
    (apply #'+ (send-all _send_twist_command :serialization-length)) 4
    ;; kortex_driver/WrenchCommand[] _send_wrench_command
    (apply #'+ (send-all _send_wrench_command :serialization-length)) 4
    ;; kortex_driver/Base_JointSpeeds[] _send_joint_speeds
    (apply #'+ (send-all _send_joint_speeds :serialization-length)) 4
    ;; kortex_driver/ConstrainedPose[] _reach_pose
    (apply #'+ (send-all _reach_pose :serialization-length)) 4
    ;; kortex_driver/ConstrainedJointAngles[] _reach_joint_angles
    (apply #'+ (send-all _reach_joint_angles :serialization-length)) 4
    ;; uint32[] _toggle_admittance_mode
    (* 4    (length _toggle_admittance_mode)) 4
    ;; kortex_driver/Snapshot[] _snapshot
    (apply #'+ (send-all _snapshot :serialization-length)) 4
    ;; kortex_driver/SwitchControlMapping[] _switch_control_mapping
    (apply #'+ (send-all _switch_control_mapping :serialization-length)) 4
    ;; uint32[] _navigate_joints
    (* 4    (length _navigate_joints)) 4
    ;; uint32[] _navigate_mappings
    (* 4    (length _navigate_mappings)) 4
    ;; kortex_driver/ChangeTwist[] _change_twist
    (apply #'+ (send-all _change_twist :serialization-length)) 4
    ;; kortex_driver/ChangeJointSpeeds[] _change_joint_speeds
    (apply #'+ (send-all _change_joint_speeds :serialization-length)) 4
    ;; kortex_driver/ChangeWrench[] _change_wrench
    (apply #'+ (send-all _change_wrench :serialization-length)) 4
    ;; kortex_driver/EmergencyStop[] _apply_emergency_stop
    (apply #'+ (send-all _apply_emergency_stop :serialization-length)) 4
    ;; kortex_driver/Faults[] _clear_faults
    (apply #'+ (send-all _clear_faults :serialization-length)) 4
    ;; kortex_driver/Delay[] _delay
    (apply #'+ (send-all _delay :serialization-length)) 4
    ;; kortex_driver/ActionHandle[] _execute_action
    (apply #'+ (send-all _execute_action :serialization-length)) 4
    ;; kortex_driver/GripperCommand[] _send_gripper_command
    (apply #'+ (send-all _send_gripper_command :serialization-length)) 4
    ;; kortex_driver/GpioCommand[] _send_gpio_command
    (apply #'+ (send-all _send_gpio_command :serialization-length)) 4
    ;; kortex_driver/Base_Stop[] _stop_action
    (apply #'+ (send-all _stop_action :serialization-length)) 4
    ;; kortex_driver/PreComputedJointTrajectory[] _play_pre_computed_trajectory
    (apply #'+ (send-all _play_pre_computed_trajectory :serialization-length)) 4
    ;; kortex_driver/SequenceHandle[] _execute_sequence
    (apply #'+ (send-all _execute_sequence :serialization-length)) 4
    ;; kortex_driver/WaypointList[] _execute_waypoint_list
    (apply #'+ (send-all _execute_waypoint_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistCommand[] _send_twist_command
     (write-long (length _send_twist_command) s)
     (dolist (elem _send_twist_command)
       (send elem :serialize s)
       )
     ;; kortex_driver/WrenchCommand[] _send_wrench_command
     (write-long (length _send_wrench_command) s)
     (dolist (elem _send_wrench_command)
       (send elem :serialize s)
       )
     ;; kortex_driver/Base_JointSpeeds[] _send_joint_speeds
     (write-long (length _send_joint_speeds) s)
     (dolist (elem _send_joint_speeds)
       (send elem :serialize s)
       )
     ;; kortex_driver/ConstrainedPose[] _reach_pose
     (write-long (length _reach_pose) s)
     (dolist (elem _reach_pose)
       (send elem :serialize s)
       )
     ;; kortex_driver/ConstrainedJointAngles[] _reach_joint_angles
     (write-long (length _reach_joint_angles) s)
     (dolist (elem _reach_joint_angles)
       (send elem :serialize s)
       )
     ;; uint32[] _toggle_admittance_mode
     (write-long (length _toggle_admittance_mode) s)
     (dotimes (i (length _toggle_admittance_mode))
       (write-long (elt _toggle_admittance_mode i) s)
       )
     ;; kortex_driver/Snapshot[] _snapshot
     (write-long (length _snapshot) s)
     (dolist (elem _snapshot)
       (send elem :serialize s)
       )
     ;; kortex_driver/SwitchControlMapping[] _switch_control_mapping
     (write-long (length _switch_control_mapping) s)
     (dolist (elem _switch_control_mapping)
       (send elem :serialize s)
       )
     ;; uint32[] _navigate_joints
     (write-long (length _navigate_joints) s)
     (dotimes (i (length _navigate_joints))
       (write-long (elt _navigate_joints i) s)
       )
     ;; uint32[] _navigate_mappings
     (write-long (length _navigate_mappings) s)
     (dotimes (i (length _navigate_mappings))
       (write-long (elt _navigate_mappings i) s)
       )
     ;; kortex_driver/ChangeTwist[] _change_twist
     (write-long (length _change_twist) s)
     (dolist (elem _change_twist)
       (send elem :serialize s)
       )
     ;; kortex_driver/ChangeJointSpeeds[] _change_joint_speeds
     (write-long (length _change_joint_speeds) s)
     (dolist (elem _change_joint_speeds)
       (send elem :serialize s)
       )
     ;; kortex_driver/ChangeWrench[] _change_wrench
     (write-long (length _change_wrench) s)
     (dolist (elem _change_wrench)
       (send elem :serialize s)
       )
     ;; kortex_driver/EmergencyStop[] _apply_emergency_stop
     (write-long (length _apply_emergency_stop) s)
     (dolist (elem _apply_emergency_stop)
       (send elem :serialize s)
       )
     ;; kortex_driver/Faults[] _clear_faults
     (write-long (length _clear_faults) s)
     (dolist (elem _clear_faults)
       (send elem :serialize s)
       )
     ;; kortex_driver/Delay[] _delay
     (write-long (length _delay) s)
     (dolist (elem _delay)
       (send elem :serialize s)
       )
     ;; kortex_driver/ActionHandle[] _execute_action
     (write-long (length _execute_action) s)
     (dolist (elem _execute_action)
       (send elem :serialize s)
       )
     ;; kortex_driver/GripperCommand[] _send_gripper_command
     (write-long (length _send_gripper_command) s)
     (dolist (elem _send_gripper_command)
       (send elem :serialize s)
       )
     ;; kortex_driver/GpioCommand[] _send_gpio_command
     (write-long (length _send_gpio_command) s)
     (dolist (elem _send_gpio_command)
       (send elem :serialize s)
       )
     ;; kortex_driver/Base_Stop[] _stop_action
     (write-long (length _stop_action) s)
     (dolist (elem _stop_action)
       (send elem :serialize s)
       )
     ;; kortex_driver/PreComputedJointTrajectory[] _play_pre_computed_trajectory
     (write-long (length _play_pre_computed_trajectory) s)
     (dolist (elem _play_pre_computed_trajectory)
       (send elem :serialize s)
       )
     ;; kortex_driver/SequenceHandle[] _execute_sequence
     (write-long (length _execute_sequence) s)
     (dolist (elem _execute_sequence)
       (send elem :serialize s)
       )
     ;; kortex_driver/WaypointList[] _execute_waypoint_list
     (write-long (length _execute_waypoint_list) s)
     (dolist (elem _execute_waypoint_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistCommand[] _send_twist_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _send_twist_command (let (r) (dotimes (i n) (push (instance kortex_driver::TwistCommand :init) r)) r))
     (dolist (elem- _send_twist_command)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/WrenchCommand[] _send_wrench_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _send_wrench_command (let (r) (dotimes (i n) (push (instance kortex_driver::WrenchCommand :init) r)) r))
     (dolist (elem- _send_wrench_command)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/Base_JointSpeeds[] _send_joint_speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _send_joint_speeds (let (r) (dotimes (i n) (push (instance kortex_driver::Base_JointSpeeds :init) r)) r))
     (dolist (elem- _send_joint_speeds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ConstrainedPose[] _reach_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reach_pose (let (r) (dotimes (i n) (push (instance kortex_driver::ConstrainedPose :init) r)) r))
     (dolist (elem- _reach_pose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ConstrainedJointAngles[] _reach_joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reach_joint_angles (let (r) (dotimes (i n) (push (instance kortex_driver::ConstrainedJointAngles :init) r)) r))
     (dolist (elem- _reach_joint_angles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32[] _toggle_admittance_mode
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _toggle_admittance_mode (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _toggle_admittance_mode i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; kortex_driver/Snapshot[] _snapshot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _snapshot (let (r) (dotimes (i n) (push (instance kortex_driver::Snapshot :init) r)) r))
     (dolist (elem- _snapshot)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/SwitchControlMapping[] _switch_control_mapping
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _switch_control_mapping (let (r) (dotimes (i n) (push (instance kortex_driver::SwitchControlMapping :init) r)) r))
     (dolist (elem- _switch_control_mapping)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32[] _navigate_joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _navigate_joints (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _navigate_joints i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _navigate_mappings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _navigate_mappings (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _navigate_mappings i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; kortex_driver/ChangeTwist[] _change_twist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _change_twist (let (r) (dotimes (i n) (push (instance kortex_driver::ChangeTwist :init) r)) r))
     (dolist (elem- _change_twist)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ChangeJointSpeeds[] _change_joint_speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _change_joint_speeds (let (r) (dotimes (i n) (push (instance kortex_driver::ChangeJointSpeeds :init) r)) r))
     (dolist (elem- _change_joint_speeds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ChangeWrench[] _change_wrench
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _change_wrench (let (r) (dotimes (i n) (push (instance kortex_driver::ChangeWrench :init) r)) r))
     (dolist (elem- _change_wrench)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/EmergencyStop[] _apply_emergency_stop
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _apply_emergency_stop (let (r) (dotimes (i n) (push (instance kortex_driver::EmergencyStop :init) r)) r))
     (dolist (elem- _apply_emergency_stop)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/Faults[] _clear_faults
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _clear_faults (let (r) (dotimes (i n) (push (instance kortex_driver::Faults :init) r)) r))
     (dolist (elem- _clear_faults)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/Delay[] _delay
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _delay (let (r) (dotimes (i n) (push (instance kortex_driver::Delay :init) r)) r))
     (dolist (elem- _delay)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ActionHandle[] _execute_action
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _execute_action (let (r) (dotimes (i n) (push (instance kortex_driver::ActionHandle :init) r)) r))
     (dolist (elem- _execute_action)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/GripperCommand[] _send_gripper_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _send_gripper_command (let (r) (dotimes (i n) (push (instance kortex_driver::GripperCommand :init) r)) r))
     (dolist (elem- _send_gripper_command)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/GpioCommand[] _send_gpio_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _send_gpio_command (let (r) (dotimes (i n) (push (instance kortex_driver::GpioCommand :init) r)) r))
     (dolist (elem- _send_gpio_command)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/Base_Stop[] _stop_action
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stop_action (let (r) (dotimes (i n) (push (instance kortex_driver::Base_Stop :init) r)) r))
     (dolist (elem- _stop_action)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/PreComputedJointTrajectory[] _play_pre_computed_trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _play_pre_computed_trajectory (let (r) (dotimes (i n) (push (instance kortex_driver::PreComputedJointTrajectory :init) r)) r))
     (dolist (elem- _play_pre_computed_trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/SequenceHandle[] _execute_sequence
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _execute_sequence (let (r) (dotimes (i n) (push (instance kortex_driver::SequenceHandle :init) r)) r))
     (dolist (elem- _execute_sequence)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/WaypointList[] _execute_waypoint_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _execute_waypoint_list (let (r) (dotimes (i n) (push (instance kortex_driver::WaypointList :init) r)) r))
     (dolist (elem- _execute_waypoint_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::Action_action_parameters :md5sum-) "3d6115bf166bbc775637a0580d1e6e6c")
(setf (get kortex_driver::Action_action_parameters :datatype-) "kortex_driver/Action_action_parameters")
(setf (get kortex_driver::Action_action_parameters :definition-)
      "
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
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
uint32 permission
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



(provide :kortex_driver/Action_action_parameters "3d6115bf166bbc775637a0580d1e6e6c")


