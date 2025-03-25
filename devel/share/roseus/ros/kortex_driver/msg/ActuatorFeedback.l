;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorFeedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorFeedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORFEEDBACK")
  (make-package "KORTEX_DRIVER::ACTUATORFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude ActuatorFeedback.msg.html


(defclass kortex_driver::ActuatorFeedback
  :super ros::object
  :slots (_command_id _status_flags _jitter_comm _position _velocity _torque _current_motor _voltage _temperature_motor _temperature_core _fault_bank_a _fault_bank_b _warning_bank_a _warning_bank_b ))

(defmethod kortex_driver::ActuatorFeedback
  (:init
   (&key
    ((:command_id __command_id) 0)
    ((:status_flags __status_flags) 0)
    ((:jitter_comm __jitter_comm) 0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:torque __torque) 0.0)
    ((:current_motor __current_motor) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature_motor __temperature_motor) 0.0)
    ((:temperature_core __temperature_core) 0.0)
    ((:fault_bank_a __fault_bank_a) 0)
    ((:fault_bank_b __fault_bank_b) 0)
    ((:warning_bank_a __warning_bank_a) 0)
    ((:warning_bank_b __warning_bank_b) 0)
    )
   (send-super :init)
   (setq _command_id (round __command_id))
   (setq _status_flags (round __status_flags))
   (setq _jitter_comm (round __jitter_comm))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _torque (float __torque))
   (setq _current_motor (float __current_motor))
   (setq _voltage (float __voltage))
   (setq _temperature_motor (float __temperature_motor))
   (setq _temperature_core (float __temperature_core))
   (setq _fault_bank_a (round __fault_bank_a))
   (setq _fault_bank_b (round __fault_bank_b))
   (setq _warning_bank_a (round __warning_bank_a))
   (setq _warning_bank_b (round __warning_bank_b))
   self)
  (:command_id
   (&optional __command_id)
   (if __command_id (setq _command_id __command_id)) _command_id)
  (:status_flags
   (&optional __status_flags)
   (if __status_flags (setq _status_flags __status_flags)) _status_flags)
  (:jitter_comm
   (&optional __jitter_comm)
   (if __jitter_comm (setq _jitter_comm __jitter_comm)) _jitter_comm)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:current_motor
   (&optional __current_motor)
   (if __current_motor (setq _current_motor __current_motor)) _current_motor)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature_motor
   (&optional __temperature_motor)
   (if __temperature_motor (setq _temperature_motor __temperature_motor)) _temperature_motor)
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
  (:serialization-length
   ()
   (+
    ;; uint32 _command_id
    4
    ;; uint32 _status_flags
    4
    ;; uint32 _jitter_comm
    4
    ;; float32 _position
    4
    ;; float32 _velocity
    4
    ;; float32 _torque
    4
    ;; float32 _current_motor
    4
    ;; float32 _voltage
    4
    ;; float32 _temperature_motor
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
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _command_id
       (write-long _command_id s)
     ;; uint32 _status_flags
       (write-long _status_flags s)
     ;; uint32 _jitter_comm
       (write-long _jitter_comm s)
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _torque
       (sys::poke _torque (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_motor
       (sys::poke _current_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature_motor
       (sys::poke _temperature_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _command_id
     (setq _command_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _status_flags
     (setq _status_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _jitter_comm
     (setq _jitter_comm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _torque
     (setq _torque (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_motor
     (setq _current_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature_motor
     (setq _temperature_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
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
   ;;
   self)
  )

(setf (get kortex_driver::ActuatorFeedback :md5sum-) "1c44a9029e2d1e74ae3d04b3c212afbd")
(setf (get kortex_driver::ActuatorFeedback :datatype-) "kortex_driver/ActuatorFeedback")
(setf (get kortex_driver::ActuatorFeedback :definition-)
      "
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
")



(provide :kortex_driver/ActuatorFeedback "1c44a9029e2d1e74ae3d04b3c212afbd")


