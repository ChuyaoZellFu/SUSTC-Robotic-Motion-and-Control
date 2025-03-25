;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BaseFeedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BaseFeedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASEFEEDBACK")
  (make-package "KORTEX_DRIVER::BASEFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude BaseFeedback.msg.html


(defclass kortex_driver::BaseFeedback
  :super ros::object
  :slots (_active_state_connection_identifier _active_state _arm_voltage _arm_current _temperature_cpu _temperature_ambient _imu_acceleration_x _imu_acceleration_y _imu_acceleration_z _imu_angular_velocity_x _imu_angular_velocity_y _imu_angular_velocity_z _tool_pose_x _tool_pose_y _tool_pose_z _tool_pose_theta_x _tool_pose_theta_y _tool_pose_theta_z _tool_twist_linear_x _tool_twist_linear_y _tool_twist_linear_z _tool_twist_angular_x _tool_twist_angular_y _tool_twist_angular_z _tool_external_wrench_force_x _tool_external_wrench_force_y _tool_external_wrench_force_z _tool_external_wrench_torque_x _tool_external_wrench_torque_y _tool_external_wrench_torque_z _fault_bank_a _fault_bank_b _warning_bank_a _warning_bank_b _commanded_tool_pose_x _commanded_tool_pose_y _commanded_tool_pose_z _commanded_tool_pose_theta_x _commanded_tool_pose_theta_y _commanded_tool_pose_theta_z ))

(defmethod kortex_driver::BaseFeedback
  (:init
   (&key
    ((:active_state_connection_identifier __active_state_connection_identifier) 0)
    ((:active_state __active_state) 0)
    ((:arm_voltage __arm_voltage) 0.0)
    ((:arm_current __arm_current) 0.0)
    ((:temperature_cpu __temperature_cpu) 0.0)
    ((:temperature_ambient __temperature_ambient) 0.0)
    ((:imu_acceleration_x __imu_acceleration_x) 0.0)
    ((:imu_acceleration_y __imu_acceleration_y) 0.0)
    ((:imu_acceleration_z __imu_acceleration_z) 0.0)
    ((:imu_angular_velocity_x __imu_angular_velocity_x) 0.0)
    ((:imu_angular_velocity_y __imu_angular_velocity_y) 0.0)
    ((:imu_angular_velocity_z __imu_angular_velocity_z) 0.0)
    ((:tool_pose_x __tool_pose_x) 0.0)
    ((:tool_pose_y __tool_pose_y) 0.0)
    ((:tool_pose_z __tool_pose_z) 0.0)
    ((:tool_pose_theta_x __tool_pose_theta_x) 0.0)
    ((:tool_pose_theta_y __tool_pose_theta_y) 0.0)
    ((:tool_pose_theta_z __tool_pose_theta_z) 0.0)
    ((:tool_twist_linear_x __tool_twist_linear_x) 0.0)
    ((:tool_twist_linear_y __tool_twist_linear_y) 0.0)
    ((:tool_twist_linear_z __tool_twist_linear_z) 0.0)
    ((:tool_twist_angular_x __tool_twist_angular_x) 0.0)
    ((:tool_twist_angular_y __tool_twist_angular_y) 0.0)
    ((:tool_twist_angular_z __tool_twist_angular_z) 0.0)
    ((:tool_external_wrench_force_x __tool_external_wrench_force_x) 0.0)
    ((:tool_external_wrench_force_y __tool_external_wrench_force_y) 0.0)
    ((:tool_external_wrench_force_z __tool_external_wrench_force_z) 0.0)
    ((:tool_external_wrench_torque_x __tool_external_wrench_torque_x) 0.0)
    ((:tool_external_wrench_torque_y __tool_external_wrench_torque_y) 0.0)
    ((:tool_external_wrench_torque_z __tool_external_wrench_torque_z) 0.0)
    ((:fault_bank_a __fault_bank_a) 0)
    ((:fault_bank_b __fault_bank_b) 0)
    ((:warning_bank_a __warning_bank_a) 0)
    ((:warning_bank_b __warning_bank_b) 0)
    ((:commanded_tool_pose_x __commanded_tool_pose_x) 0.0)
    ((:commanded_tool_pose_y __commanded_tool_pose_y) 0.0)
    ((:commanded_tool_pose_z __commanded_tool_pose_z) 0.0)
    ((:commanded_tool_pose_theta_x __commanded_tool_pose_theta_x) 0.0)
    ((:commanded_tool_pose_theta_y __commanded_tool_pose_theta_y) 0.0)
    ((:commanded_tool_pose_theta_z __commanded_tool_pose_theta_z) 0.0)
    )
   (send-super :init)
   (setq _active_state_connection_identifier (round __active_state_connection_identifier))
   (setq _active_state (round __active_state))
   (setq _arm_voltage (float __arm_voltage))
   (setq _arm_current (float __arm_current))
   (setq _temperature_cpu (float __temperature_cpu))
   (setq _temperature_ambient (float __temperature_ambient))
   (setq _imu_acceleration_x (float __imu_acceleration_x))
   (setq _imu_acceleration_y (float __imu_acceleration_y))
   (setq _imu_acceleration_z (float __imu_acceleration_z))
   (setq _imu_angular_velocity_x (float __imu_angular_velocity_x))
   (setq _imu_angular_velocity_y (float __imu_angular_velocity_y))
   (setq _imu_angular_velocity_z (float __imu_angular_velocity_z))
   (setq _tool_pose_x (float __tool_pose_x))
   (setq _tool_pose_y (float __tool_pose_y))
   (setq _tool_pose_z (float __tool_pose_z))
   (setq _tool_pose_theta_x (float __tool_pose_theta_x))
   (setq _tool_pose_theta_y (float __tool_pose_theta_y))
   (setq _tool_pose_theta_z (float __tool_pose_theta_z))
   (setq _tool_twist_linear_x (float __tool_twist_linear_x))
   (setq _tool_twist_linear_y (float __tool_twist_linear_y))
   (setq _tool_twist_linear_z (float __tool_twist_linear_z))
   (setq _tool_twist_angular_x (float __tool_twist_angular_x))
   (setq _tool_twist_angular_y (float __tool_twist_angular_y))
   (setq _tool_twist_angular_z (float __tool_twist_angular_z))
   (setq _tool_external_wrench_force_x (float __tool_external_wrench_force_x))
   (setq _tool_external_wrench_force_y (float __tool_external_wrench_force_y))
   (setq _tool_external_wrench_force_z (float __tool_external_wrench_force_z))
   (setq _tool_external_wrench_torque_x (float __tool_external_wrench_torque_x))
   (setq _tool_external_wrench_torque_y (float __tool_external_wrench_torque_y))
   (setq _tool_external_wrench_torque_z (float __tool_external_wrench_torque_z))
   (setq _fault_bank_a (round __fault_bank_a))
   (setq _fault_bank_b (round __fault_bank_b))
   (setq _warning_bank_a (round __warning_bank_a))
   (setq _warning_bank_b (round __warning_bank_b))
   (setq _commanded_tool_pose_x (float __commanded_tool_pose_x))
   (setq _commanded_tool_pose_y (float __commanded_tool_pose_y))
   (setq _commanded_tool_pose_z (float __commanded_tool_pose_z))
   (setq _commanded_tool_pose_theta_x (float __commanded_tool_pose_theta_x))
   (setq _commanded_tool_pose_theta_y (float __commanded_tool_pose_theta_y))
   (setq _commanded_tool_pose_theta_z (float __commanded_tool_pose_theta_z))
   self)
  (:active_state_connection_identifier
   (&optional __active_state_connection_identifier)
   (if __active_state_connection_identifier (setq _active_state_connection_identifier __active_state_connection_identifier)) _active_state_connection_identifier)
  (:active_state
   (&optional __active_state)
   (if __active_state (setq _active_state __active_state)) _active_state)
  (:arm_voltage
   (&optional __arm_voltage)
   (if __arm_voltage (setq _arm_voltage __arm_voltage)) _arm_voltage)
  (:arm_current
   (&optional __arm_current)
   (if __arm_current (setq _arm_current __arm_current)) _arm_current)
  (:temperature_cpu
   (&optional __temperature_cpu)
   (if __temperature_cpu (setq _temperature_cpu __temperature_cpu)) _temperature_cpu)
  (:temperature_ambient
   (&optional __temperature_ambient)
   (if __temperature_ambient (setq _temperature_ambient __temperature_ambient)) _temperature_ambient)
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
  (:tool_pose_x
   (&optional __tool_pose_x)
   (if __tool_pose_x (setq _tool_pose_x __tool_pose_x)) _tool_pose_x)
  (:tool_pose_y
   (&optional __tool_pose_y)
   (if __tool_pose_y (setq _tool_pose_y __tool_pose_y)) _tool_pose_y)
  (:tool_pose_z
   (&optional __tool_pose_z)
   (if __tool_pose_z (setq _tool_pose_z __tool_pose_z)) _tool_pose_z)
  (:tool_pose_theta_x
   (&optional __tool_pose_theta_x)
   (if __tool_pose_theta_x (setq _tool_pose_theta_x __tool_pose_theta_x)) _tool_pose_theta_x)
  (:tool_pose_theta_y
   (&optional __tool_pose_theta_y)
   (if __tool_pose_theta_y (setq _tool_pose_theta_y __tool_pose_theta_y)) _tool_pose_theta_y)
  (:tool_pose_theta_z
   (&optional __tool_pose_theta_z)
   (if __tool_pose_theta_z (setq _tool_pose_theta_z __tool_pose_theta_z)) _tool_pose_theta_z)
  (:tool_twist_linear_x
   (&optional __tool_twist_linear_x)
   (if __tool_twist_linear_x (setq _tool_twist_linear_x __tool_twist_linear_x)) _tool_twist_linear_x)
  (:tool_twist_linear_y
   (&optional __tool_twist_linear_y)
   (if __tool_twist_linear_y (setq _tool_twist_linear_y __tool_twist_linear_y)) _tool_twist_linear_y)
  (:tool_twist_linear_z
   (&optional __tool_twist_linear_z)
   (if __tool_twist_linear_z (setq _tool_twist_linear_z __tool_twist_linear_z)) _tool_twist_linear_z)
  (:tool_twist_angular_x
   (&optional __tool_twist_angular_x)
   (if __tool_twist_angular_x (setq _tool_twist_angular_x __tool_twist_angular_x)) _tool_twist_angular_x)
  (:tool_twist_angular_y
   (&optional __tool_twist_angular_y)
   (if __tool_twist_angular_y (setq _tool_twist_angular_y __tool_twist_angular_y)) _tool_twist_angular_y)
  (:tool_twist_angular_z
   (&optional __tool_twist_angular_z)
   (if __tool_twist_angular_z (setq _tool_twist_angular_z __tool_twist_angular_z)) _tool_twist_angular_z)
  (:tool_external_wrench_force_x
   (&optional __tool_external_wrench_force_x)
   (if __tool_external_wrench_force_x (setq _tool_external_wrench_force_x __tool_external_wrench_force_x)) _tool_external_wrench_force_x)
  (:tool_external_wrench_force_y
   (&optional __tool_external_wrench_force_y)
   (if __tool_external_wrench_force_y (setq _tool_external_wrench_force_y __tool_external_wrench_force_y)) _tool_external_wrench_force_y)
  (:tool_external_wrench_force_z
   (&optional __tool_external_wrench_force_z)
   (if __tool_external_wrench_force_z (setq _tool_external_wrench_force_z __tool_external_wrench_force_z)) _tool_external_wrench_force_z)
  (:tool_external_wrench_torque_x
   (&optional __tool_external_wrench_torque_x)
   (if __tool_external_wrench_torque_x (setq _tool_external_wrench_torque_x __tool_external_wrench_torque_x)) _tool_external_wrench_torque_x)
  (:tool_external_wrench_torque_y
   (&optional __tool_external_wrench_torque_y)
   (if __tool_external_wrench_torque_y (setq _tool_external_wrench_torque_y __tool_external_wrench_torque_y)) _tool_external_wrench_torque_y)
  (:tool_external_wrench_torque_z
   (&optional __tool_external_wrench_torque_z)
   (if __tool_external_wrench_torque_z (setq _tool_external_wrench_torque_z __tool_external_wrench_torque_z)) _tool_external_wrench_torque_z)
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
  (:commanded_tool_pose_x
   (&optional __commanded_tool_pose_x)
   (if __commanded_tool_pose_x (setq _commanded_tool_pose_x __commanded_tool_pose_x)) _commanded_tool_pose_x)
  (:commanded_tool_pose_y
   (&optional __commanded_tool_pose_y)
   (if __commanded_tool_pose_y (setq _commanded_tool_pose_y __commanded_tool_pose_y)) _commanded_tool_pose_y)
  (:commanded_tool_pose_z
   (&optional __commanded_tool_pose_z)
   (if __commanded_tool_pose_z (setq _commanded_tool_pose_z __commanded_tool_pose_z)) _commanded_tool_pose_z)
  (:commanded_tool_pose_theta_x
   (&optional __commanded_tool_pose_theta_x)
   (if __commanded_tool_pose_theta_x (setq _commanded_tool_pose_theta_x __commanded_tool_pose_theta_x)) _commanded_tool_pose_theta_x)
  (:commanded_tool_pose_theta_y
   (&optional __commanded_tool_pose_theta_y)
   (if __commanded_tool_pose_theta_y (setq _commanded_tool_pose_theta_y __commanded_tool_pose_theta_y)) _commanded_tool_pose_theta_y)
  (:commanded_tool_pose_theta_z
   (&optional __commanded_tool_pose_theta_z)
   (if __commanded_tool_pose_theta_z (setq _commanded_tool_pose_theta_z __commanded_tool_pose_theta_z)) _commanded_tool_pose_theta_z)
  (:serialization-length
   ()
   (+
    ;; uint32 _active_state_connection_identifier
    4
    ;; uint32 _active_state
    4
    ;; float32 _arm_voltage
    4
    ;; float32 _arm_current
    4
    ;; float32 _temperature_cpu
    4
    ;; float32 _temperature_ambient
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
    ;; float32 _tool_pose_x
    4
    ;; float32 _tool_pose_y
    4
    ;; float32 _tool_pose_z
    4
    ;; float32 _tool_pose_theta_x
    4
    ;; float32 _tool_pose_theta_y
    4
    ;; float32 _tool_pose_theta_z
    4
    ;; float32 _tool_twist_linear_x
    4
    ;; float32 _tool_twist_linear_y
    4
    ;; float32 _tool_twist_linear_z
    4
    ;; float32 _tool_twist_angular_x
    4
    ;; float32 _tool_twist_angular_y
    4
    ;; float32 _tool_twist_angular_z
    4
    ;; float32 _tool_external_wrench_force_x
    4
    ;; float32 _tool_external_wrench_force_y
    4
    ;; float32 _tool_external_wrench_force_z
    4
    ;; float32 _tool_external_wrench_torque_x
    4
    ;; float32 _tool_external_wrench_torque_y
    4
    ;; float32 _tool_external_wrench_torque_z
    4
    ;; uint32 _fault_bank_a
    4
    ;; uint32 _fault_bank_b
    4
    ;; uint32 _warning_bank_a
    4
    ;; uint32 _warning_bank_b
    4
    ;; float32 _commanded_tool_pose_x
    4
    ;; float32 _commanded_tool_pose_y
    4
    ;; float32 _commanded_tool_pose_z
    4
    ;; float32 _commanded_tool_pose_theta_x
    4
    ;; float32 _commanded_tool_pose_theta_y
    4
    ;; float32 _commanded_tool_pose_theta_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _active_state_connection_identifier
       (write-long _active_state_connection_identifier s)
     ;; uint32 _active_state
       (write-long _active_state s)
     ;; float32 _arm_voltage
       (sys::poke _arm_voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arm_current
       (sys::poke _arm_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature_cpu
       (sys::poke _temperature_cpu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature_ambient
       (sys::poke _temperature_ambient (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
     ;; float32 _tool_pose_x
       (sys::poke _tool_pose_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_pose_y
       (sys::poke _tool_pose_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_pose_z
       (sys::poke _tool_pose_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_pose_theta_x
       (sys::poke _tool_pose_theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_pose_theta_y
       (sys::poke _tool_pose_theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_pose_theta_z
       (sys::poke _tool_pose_theta_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_linear_x
       (sys::poke _tool_twist_linear_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_linear_y
       (sys::poke _tool_twist_linear_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_linear_z
       (sys::poke _tool_twist_linear_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_angular_x
       (sys::poke _tool_twist_angular_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_angular_y
       (sys::poke _tool_twist_angular_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_twist_angular_z
       (sys::poke _tool_twist_angular_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_force_x
       (sys::poke _tool_external_wrench_force_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_force_y
       (sys::poke _tool_external_wrench_force_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_force_z
       (sys::poke _tool_external_wrench_force_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_torque_x
       (sys::poke _tool_external_wrench_torque_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_torque_y
       (sys::poke _tool_external_wrench_torque_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tool_external_wrench_torque_z
       (sys::poke _tool_external_wrench_torque_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _fault_bank_a
       (write-long _fault_bank_a s)
     ;; uint32 _fault_bank_b
       (write-long _fault_bank_b s)
     ;; uint32 _warning_bank_a
       (write-long _warning_bank_a s)
     ;; uint32 _warning_bank_b
       (write-long _warning_bank_b s)
     ;; float32 _commanded_tool_pose_x
       (sys::poke _commanded_tool_pose_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _commanded_tool_pose_y
       (sys::poke _commanded_tool_pose_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _commanded_tool_pose_z
       (sys::poke _commanded_tool_pose_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _commanded_tool_pose_theta_x
       (sys::poke _commanded_tool_pose_theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _commanded_tool_pose_theta_y
       (sys::poke _commanded_tool_pose_theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _commanded_tool_pose_theta_z
       (sys::poke _commanded_tool_pose_theta_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _active_state_connection_identifier
     (setq _active_state_connection_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _active_state
     (setq _active_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _arm_voltage
     (setq _arm_voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arm_current
     (setq _arm_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature_cpu
     (setq _temperature_cpu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature_ambient
     (setq _temperature_ambient (sys::peek buf ptr- :float)) (incf ptr- 4)
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
   ;; float32 _tool_pose_x
     (setq _tool_pose_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_pose_y
     (setq _tool_pose_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_pose_z
     (setq _tool_pose_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_pose_theta_x
     (setq _tool_pose_theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_pose_theta_y
     (setq _tool_pose_theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_pose_theta_z
     (setq _tool_pose_theta_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_linear_x
     (setq _tool_twist_linear_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_linear_y
     (setq _tool_twist_linear_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_linear_z
     (setq _tool_twist_linear_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_angular_x
     (setq _tool_twist_angular_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_angular_y
     (setq _tool_twist_angular_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_twist_angular_z
     (setq _tool_twist_angular_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_force_x
     (setq _tool_external_wrench_force_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_force_y
     (setq _tool_external_wrench_force_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_force_z
     (setq _tool_external_wrench_force_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_torque_x
     (setq _tool_external_wrench_torque_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_torque_y
     (setq _tool_external_wrench_torque_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tool_external_wrench_torque_z
     (setq _tool_external_wrench_torque_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _fault_bank_a
     (setq _fault_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _fault_bank_b
     (setq _fault_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_a
     (setq _warning_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_b
     (setq _warning_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_x
     (setq _commanded_tool_pose_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_y
     (setq _commanded_tool_pose_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_z
     (setq _commanded_tool_pose_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_theta_x
     (setq _commanded_tool_pose_theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_theta_y
     (setq _commanded_tool_pose_theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _commanded_tool_pose_theta_z
     (setq _commanded_tool_pose_theta_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::BaseFeedback :md5sum-) "10860d64294ad841b25c45c5195aa64d")
(setf (get kortex_driver::BaseFeedback :datatype-) "kortex_driver/BaseFeedback")
(setf (get kortex_driver::BaseFeedback :definition-)
      "
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
")



(provide :kortex_driver/BaseFeedback "10860d64294ad841b25c45c5195aa64d")


