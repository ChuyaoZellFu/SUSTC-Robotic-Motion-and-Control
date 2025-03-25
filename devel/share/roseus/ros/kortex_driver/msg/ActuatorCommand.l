;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCOMMAND")
  (make-package "KORTEX_DRIVER::ACTUATORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ActuatorCommand.msg.html


(defclass kortex_driver::ActuatorCommand
  :super ros::object
  :slots (_command_id _flags _position _velocity _torque_joint _current_motor ))

(defmethod kortex_driver::ActuatorCommand
  (:init
   (&key
    ((:command_id __command_id) 0)
    ((:flags __flags) 0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:torque_joint __torque_joint) 0.0)
    ((:current_motor __current_motor) 0.0)
    )
   (send-super :init)
   (setq _command_id (round __command_id))
   (setq _flags (round __flags))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _torque_joint (float __torque_joint))
   (setq _current_motor (float __current_motor))
   self)
  (:command_id
   (&optional __command_id)
   (if __command_id (setq _command_id __command_id)) _command_id)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:torque_joint
   (&optional __torque_joint)
   (if __torque_joint (setq _torque_joint __torque_joint)) _torque_joint)
  (:current_motor
   (&optional __current_motor)
   (if __current_motor (setq _current_motor __current_motor)) _current_motor)
  (:serialization-length
   ()
   (+
    ;; uint32 _command_id
    4
    ;; uint32 _flags
    4
    ;; float32 _position
    4
    ;; float32 _velocity
    4
    ;; float32 _torque_joint
    4
    ;; float32 _current_motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _command_id
       (write-long _command_id s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _torque_joint
       (sys::poke _torque_joint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_motor
       (sys::poke _current_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _command_id
     (setq _command_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _torque_joint
     (setq _torque_joint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_motor
     (setq _current_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ActuatorCommand :md5sum-) "2d6d46bc48110c570d5a382cb600d4f7")
(setf (get kortex_driver::ActuatorCommand :datatype-) "kortex_driver/ActuatorCommand")
(setf (get kortex_driver::ActuatorCommand :definition-)
      "
uint32 command_id
uint32 flags
float32 position
float32 velocity
float32 torque_joint
float32 current_motor
")



(provide :kortex_driver/ActuatorCommand "2d6d46bc48110c570d5a382cb600d4f7")


