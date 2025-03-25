;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_Feedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_Feedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_FEEDBACK")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_Feedback.msg.html


(defclass kortex_driver::InterconnectCyclic_Feedback
  :super ros::object
  :slots (_feedback_id _status_flags _jitter_comm _imu_acceleration_x _imu_acceleration_y _imu_acceleration_z _imu_angular_velocity_x _imu_angular_velocity_y _imu_angular_velocity_z _voltage _temperature_core _fault_bank_a _fault_bank_b _warning_bank_a _warning_bank_b _oneof_tool_feedback ))

(defmethod kortex_driver::InterconnectCyclic_Feedback
  (:init
   (&key
    ((:feedback_id __feedback_id) (instance kortex_driver::InterconnectCyclic_MessageId :init))
    ((:status_flags __status_flags) 0)
    ((:jitter_comm __jitter_comm) 0)
    ((:imu_acceleration_x __imu_acceleration_x) 0.0)
    ((:imu_acceleration_y __imu_acceleration_y) 0.0)
    ((:imu_acceleration_z __imu_acceleration_z) 0.0)
    ((:imu_angular_velocity_x __imu_angular_velocity_x) 0.0)
    ((:imu_angular_velocity_y __imu_angular_velocity_y) 0.0)
    ((:imu_angular_velocity_z __imu_angular_velocity_z) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature_core __temperature_core) 0.0)
    ((:fault_bank_a __fault_bank_a) 0)
    ((:fault_bank_b __fault_bank_b) 0)
    ((:warning_bank_a __warning_bank_a) 0)
    ((:warning_bank_b __warning_bank_b) 0)
    ((:oneof_tool_feedback __oneof_tool_feedback) (instance kortex_driver::InterconnectCyclic_Feedback_tool_feedback :init))
    )
   (send-super :init)
   (setq _feedback_id __feedback_id)
   (setq _status_flags (round __status_flags))
   (setq _jitter_comm (round __jitter_comm))
   (setq _imu_acceleration_x (float __imu_acceleration_x))
   (setq _imu_acceleration_y (float __imu_acceleration_y))
   (setq _imu_acceleration_z (float __imu_acceleration_z))
   (setq _imu_angular_velocity_x (float __imu_angular_velocity_x))
   (setq _imu_angular_velocity_y (float __imu_angular_velocity_y))
   (setq _imu_angular_velocity_z (float __imu_angular_velocity_z))
   (setq _voltage (float __voltage))
   (setq _temperature_core (float __temperature_core))
   (setq _fault_bank_a (round __fault_bank_a))
   (setq _fault_bank_b (round __fault_bank_b))
   (setq _warning_bank_a (round __warning_bank_a))
   (setq _warning_bank_b (round __warning_bank_b))
   (setq _oneof_tool_feedback __oneof_tool_feedback)
   self)
  (:feedback_id
   (&rest __feedback_id)
   (if (keywordp (car __feedback_id))
       (send* _feedback_id __feedback_id)
     (progn
       (if __feedback_id (setq _feedback_id (car __feedback_id)))
       _feedback_id)))
  (:status_flags
   (&optional __status_flags)
   (if __status_flags (setq _status_flags __status_flags)) _status_flags)
  (:jitter_comm
   (&optional __jitter_comm)
   (if __jitter_comm (setq _jitter_comm __jitter_comm)) _jitter_comm)
  (:imu_acceleration_x
   (&optional __imu_acceleration_x)
   (if __imu_acceleration_x (setq _imu_acceleration_x __imu_acceleration_x)) _imu_acceleration_x)
  (:imu_acceleration_y
   (&optional __imu_acceleration_y)
   (if __imu_acceleration_y (setq _imu_acceleration_y __imu_acceleration_y)) _imu_acceleration_y)
  (:imu_acceleration_z
   (&optional __imu_acceleration_z)
   (if __imu_acceleration_z (setq _imu_acceleration_z __imu_acceleration_z)) _imu_acceleration_z)
  (:imu_angular_velocity_x
   (&optional __imu_angular_velocity_x)
   (if __imu_angular_velocity_x (setq _imu_angular_velocity_x __imu_angular_velocity_x)) _imu_angular_velocity_x)
  (:imu_angular_velocity_y
   (&optional __imu_angular_velocity_y)
   (if __imu_angular_velocity_y (setq _imu_angular_velocity_y __imu_angular_velocity_y)) _imu_angular_velocity_y)
  (:imu_angular_velocity_z
   (&optional __imu_angular_velocity_z)
   (if __imu_angular_velocity_z (setq _imu_angular_velocity_z __imu_angular_velocity_z)) _imu_angular_velocity_z)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature_core
   (&optional __temperature_core)
   (if __temperature_core (setq _temperature_core __temperature_core)) _temperature_core)
  (:fault_bank_a
   (&optional __fault_bank_a)
   (if __fault_bank_a (setq _fault_bank_a __fault_bank_a)) _fault_bank_a)
  (:fault_bank_b
   (&optional __fault_bank_b)
   (if __fault_bank_b (setq _fault_bank_b __fault_bank_b)) _fault_bank_b)
  (:warning_bank_a
   (&optional __warning_bank_a)
   (if __warning_bank_a (setq _warning_bank_a __warning_bank_a)) _warning_bank_a)
  (:warning_bank_b
   (&optional __warning_bank_b)
   (if __warning_bank_b (setq _warning_bank_b __warning_bank_b)) _warning_bank_b)
  (:oneof_tool_feedback
   (&rest __oneof_tool_feedback)
   (if (keywordp (car __oneof_tool_feedback))
       (send* _oneof_tool_feedback __oneof_tool_feedback)
     (progn
       (if __oneof_tool_feedback (setq _oneof_tool_feedback (car __oneof_tool_feedback)))
       _oneof_tool_feedback)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/InterconnectCyclic_MessageId _feedback_id
    (send _feedback_id :serialization-length)
    ;; uint32 _status_flags
    4
    ;; uint32 _jitter_comm
    4
    ;; float32 _imu_acceleration_x
    4
    ;; float32 _imu_acceleration_y
    4
    ;; float32 _imu_acceleration_z
    4
    ;; float32 _imu_angular_velocity_x
    4
    ;; float32 _imu_angular_velocity_y
    4
    ;; float32 _imu_angular_velocity_z
    4
    ;; float32 _voltage
    4
    ;; float32 _temperature_core
    4
    ;; uint32 _fault_bank_a
    4
    ;; uint32 _fault_bank_b
    4
    ;; uint32 _warning_bank_a
    4
    ;; uint32 _warning_bank_b
    4
    ;; kortex_driver/InterconnectCyclic_Feedback_tool_feedback _oneof_tool_feedback
    (send _oneof_tool_feedback :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/InterconnectCyclic_MessageId _feedback_id
       (send _feedback_id :serialize s)
     ;; uint32 _status_flags
       (write-long _status_flags s)
     ;; uint32 _jitter_comm
       (write-long _jitter_comm s)
     ;; float32 _imu_acceleration_x
       (sys::poke _imu_acceleration_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_acceleration_y
       (sys::poke _imu_acceleration_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_acceleration_z
       (sys::poke _imu_acceleration_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_angular_velocity_x
       (sys::poke _imu_angular_velocity_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_angular_velocity_y
       (sys::poke _imu_angular_velocity_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_angular_velocity_z
       (sys::poke _imu_angular_velocity_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature_core
       (sys::poke _temperature_core (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _fault_bank_a
       (write-long _fault_bank_a s)
     ;; uint32 _fault_bank_b
       (write-long _fault_bank_b s)
     ;; uint32 _warning_bank_a
       (write-long _warning_bank_a s)
     ;; uint32 _warning_bank_b
       (write-long _warning_bank_b s)
     ;; kortex_driver/InterconnectCyclic_Feedback_tool_feedback _oneof_tool_feedback
       (send _oneof_tool_feedback :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/InterconnectCyclic_MessageId _feedback_id
     (send _feedback_id :deserialize buf ptr-) (incf ptr- (send _feedback_id :serialization-length))
   ;; uint32 _status_flags
     (setq _status_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _jitter_comm
     (setq _jitter_comm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _imu_acceleration_x
     (setq _imu_acceleration_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_acceleration_y
     (setq _imu_acceleration_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_acceleration_z
     (setq _imu_acceleration_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_angular_velocity_x
     (setq _imu_angular_velocity_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_angular_velocity_y
     (setq _imu_angular_velocity_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_angular_velocity_z
     (setq _imu_angular_velocity_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature_core
     (setq _temperature_core (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _fault_bank_a
     (setq _fault_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _fault_bank_b
     (setq _fault_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_a
     (setq _warning_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_b
     (setq _warning_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/InterconnectCyclic_Feedback_tool_feedback _oneof_tool_feedback
     (send _oneof_tool_feedback :deserialize buf ptr-) (incf ptr- (send _oneof_tool_feedback :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_Feedback :md5sum-) "927bf07b9c0ece3890499e76c04f37e4")
(setf (get kortex_driver::InterconnectCyclic_Feedback :datatype-) "kortex_driver/InterconnectCyclic_Feedback")
(setf (get kortex_driver::InterconnectCyclic_Feedback :definition-)
      "
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



(provide :kortex_driver/InterconnectCyclic_Feedback "927bf07b9c0ece3890499e76c04f37e4")


