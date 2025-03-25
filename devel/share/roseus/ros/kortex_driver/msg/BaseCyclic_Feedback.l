;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BaseCyclic_Feedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BaseCyclic_Feedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASECYCLIC_FEEDBACK")
  (make-package "KORTEX_DRIVER::BASECYCLIC_FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude BaseCyclic_Feedback.msg.html


(defclass kortex_driver::BaseCyclic_Feedback
  :super ros::object
  :slots (_frame_id _base _actuators _interconnect ))

(defmethod kortex_driver::BaseCyclic_Feedback
  (:init
   (&key
    ((:frame_id __frame_id) 0)
    ((:base __base) (instance kortex_driver::BaseFeedback :init))
    ((:actuators __actuators) ())
    ((:interconnect __interconnect) (instance kortex_driver::InterconnectCyclic_Feedback :init))
    )
   (send-super :init)
   (setq _frame_id (round __frame_id))
   (setq _base __base)
   (setq _actuators __actuators)
   (setq _interconnect __interconnect)
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:base
   (&rest __base)
   (if (keywordp (car __base))
       (send* _base __base)
     (progn
       (if __base (setq _base (car __base)))
       _base)))
  (:actuators
   (&rest __actuators)
   (if (keywordp (car __actuators))
       (send* _actuators __actuators)
     (progn
       (if __actuators (setq _actuators (car __actuators)))
       _actuators)))
  (:interconnect
   (&rest __interconnect)
   (if (keywordp (car __interconnect))
       (send* _interconnect __interconnect)
     (progn
       (if __interconnect (setq _interconnect (car __interconnect)))
       _interconnect)))
  (:serialization-length
   ()
   (+
    ;; uint32 _frame_id
    4
    ;; kortex_driver/BaseFeedback _base
    (send _base :serialization-length)
    ;; kortex_driver/ActuatorFeedback[] _actuators
    (apply #'+ (send-all _actuators :serialization-length)) 4
    ;; kortex_driver/InterconnectCyclic_Feedback _interconnect
    (send _interconnect :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _frame_id
       (write-long _frame_id s)
     ;; kortex_driver/BaseFeedback _base
       (send _base :serialize s)
     ;; kortex_driver/ActuatorFeedback[] _actuators
     (write-long (length _actuators) s)
     (dolist (elem _actuators)
       (send elem :serialize s)
       )
     ;; kortex_driver/InterconnectCyclic_Feedback _interconnect
       (send _interconnect :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _frame_id
     (setq _frame_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/BaseFeedback _base
     (send _base :deserialize buf ptr-) (incf ptr- (send _base :serialization-length))
   ;; kortex_driver/ActuatorFeedback[] _actuators
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actuators (let (r) (dotimes (i n) (push (instance kortex_driver::ActuatorFeedback :init) r)) r))
     (dolist (elem- _actuators)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/InterconnectCyclic_Feedback _interconnect
     (send _interconnect :deserialize buf ptr-) (incf ptr- (send _interconnect :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::BaseCyclic_Feedback :md5sum-) "5f56809f61aebc2804399d7366b38f65")
(setf (get kortex_driver::BaseCyclic_Feedback :datatype-) "kortex_driver/BaseCyclic_Feedback")
(setf (get kortex_driver::BaseCyclic_Feedback :definition-)
      "
uint32 frame_id
BaseFeedback base
ActuatorFeedback[] actuators
InterconnectCyclic_Feedback interconnect
================================================================================
MSG: kortex_driver/BaseFeedback

uint32 active_state_connection_identifier
uint32 active_state
float32 arm_voltage
float32 arm_current
float32 temperature_cpu
float32 temperature_ambient
float32 imu_acceleration_x
float32 imu_acceleration_y
float32 imu_acceleration_z
float32 imu_angular_velocity_x
float32 imu_angular_velocity_y
float32 imu_angular_velocity_z
float32 tool_pose_x
float32 tool_pose_y
float32 tool_pose_z
float32 tool_pose_theta_x
float32 tool_pose_theta_y
float32 tool_pose_theta_z
float32 tool_twist_linear_x
float32 tool_twist_linear_y
float32 tool_twist_linear_z
float32 tool_twist_angular_x
float32 tool_twist_angular_y
float32 tool_twist_angular_z
float32 tool_external_wrench_force_x
float32 tool_external_wrench_force_y
float32 tool_external_wrench_force_z
float32 tool_external_wrench_torque_x
float32 tool_external_wrench_torque_y
float32 tool_external_wrench_torque_z
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
float32 commanded_tool_pose_x
float32 commanded_tool_pose_y
float32 commanded_tool_pose_z
float32 commanded_tool_pose_theta_x
float32 commanded_tool_pose_theta_y
float32 commanded_tool_pose_theta_z
================================================================================
MSG: kortex_driver/ActuatorFeedback

uint32 command_id
uint32 status_flags
uint32 jitter_comm
float32 position
float32 velocity
float32 torque
float32 current_motor
float32 voltage
float32 temperature_motor
float32 temperature_core
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
================================================================================
MSG: kortex_driver/InterconnectCyclic_Feedback

InterconnectCyclic_MessageId feedback_id
uint32 status_flags
uint32 jitter_comm
float32 imu_acceleration_x
float32 imu_acceleration_y
float32 imu_acceleration_z
float32 imu_angular_velocity_x
float32 imu_angular_velocity_y
float32 imu_angular_velocity_z
float32 voltage
float32 temperature_core
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback
================================================================================
MSG: kortex_driver/InterconnectCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback

GripperCyclic_Feedback[] gripper_feedback
================================================================================
MSG: kortex_driver/GripperCyclic_Feedback

GripperCyclic_MessageId feedback_id
uint32 status_flags
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
MotorFeedback[] motor
================================================================================
MSG: kortex_driver/GripperCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/MotorFeedback

uint32 motor_id
float32 position
float32 velocity
float32 current_motor
float32 voltage
float32 temperature_motor
")



(provide :kortex_driver/BaseCyclic_Feedback "5f56809f61aebc2804399d7366b38f65")


