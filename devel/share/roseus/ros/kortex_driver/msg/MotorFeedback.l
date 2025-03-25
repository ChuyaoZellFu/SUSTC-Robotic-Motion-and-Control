;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MotorFeedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MotorFeedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MOTORFEEDBACK")
  (make-package "KORTEX_DRIVER::MOTORFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude MotorFeedback.msg.html


(defclass kortex_driver::MotorFeedback
  :super ros::object
  :slots (_motor_id _position _velocity _current_motor _voltage _temperature_motor ))

(defmethod kortex_driver::MotorFeedback
  (:init
   (&key
    ((:motor_id __motor_id) 0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:current_motor __current_motor) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature_motor __temperature_motor) 0.0)
    )
   (send-super :init)
   (setq _motor_id (round __motor_id))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _current_motor (float __current_motor))
   (setq _voltage (float __voltage))
   (setq _temperature_motor (float __temperature_motor))
   self)
  (:motor_id
   (&optional __motor_id)
   (if __motor_id (setq _motor_id __motor_id)) _motor_id)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:current_motor
   (&optional __current_motor)
   (if __current_motor (setq _current_motor __current_motor)) _current_motor)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature_motor
   (&optional __temperature_motor)
   (if __temperature_motor (setq _temperature_motor __temperature_motor)) _temperature_motor)
  (:serialization-length
   ()
   (+
    ;; uint32 _motor_id
    4
    ;; float32 _position
    4
    ;; float32 _velocity
    4
    ;; float32 _current_motor
    4
    ;; float32 _voltage
    4
    ;; float32 _temperature_motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _motor_id
       (write-long _motor_id s)
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_motor
       (sys::poke _current_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature_motor
       (sys::poke _temperature_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _motor_id
     (setq _motor_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_motor
     (setq _current_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature_motor
     (setq _temperature_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::MotorFeedback :md5sum-) "402f621d5644a50bc85426f20a468621")
(setf (get kortex_driver::MotorFeedback :datatype-) "kortex_driver/MotorFeedback")
(setf (get kortex_driver::MotorFeedback :definition-)
      "
uint32 motor_id
float32 position
float32 velocity
float32 current_motor
float32 voltage
float32 temperature_motor
")



(provide :kortex_driver/MotorFeedback "402f621d5644a50bc85426f20a468621")


