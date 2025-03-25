;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MotorCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MotorCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MOTORCOMMAND")
  (make-package "KORTEX_DRIVER::MOTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MotorCommand.msg.html


(defclass kortex_driver::MotorCommand
  :super ros::object
  :slots (_motor_id _position _velocity _force ))

(defmethod kortex_driver::MotorCommand
  (:init
   (&key
    ((:motor_id __motor_id) 0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:force __force) 0.0)
    )
   (send-super :init)
   (setq _motor_id (round __motor_id))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _force (float __force))
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
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:serialization-length
   ()
   (+
    ;; uint32 _motor_id
    4
    ;; float32 _position
    4
    ;; float32 _velocity
    4
    ;; float32 _force
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
     ;; float32 _force
       (sys::poke _force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _force
     (setq _force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::MotorCommand :md5sum-) "674b1a78bcad58fdb60d71acf839bd2f")
(setf (get kortex_driver::MotorCommand :datatype-) "kortex_driver/MotorCommand")
(setf (get kortex_driver::MotorCommand :definition-)
      "
uint32 motor_id
float32 position
float32 velocity
float32 force
")



(provide :kortex_driver/MotorCommand "674b1a78bcad58fdb60d71acf839bd2f")


