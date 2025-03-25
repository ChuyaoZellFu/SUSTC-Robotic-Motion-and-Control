;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PositionCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PositionCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::POSITIONCOMMAND")
  (make-package "KORTEX_DRIVER::POSITIONCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude PositionCommand.msg.html


(defclass kortex_driver::PositionCommand
  :super ros::object
  :slots (_position _velocity _acceleration ))

(defmethod kortex_driver::PositionCommand
  (:init
   (&key
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:acceleration __acceleration) 0.0)
    )
   (send-super :init)
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _acceleration (float __acceleration))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; float32 _position
    4
    ;; float32 _velocity
    4
    ;; float32 _acceleration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::PositionCommand :md5sum-) "2a6d1bc721a1409197a45acce7ca57d7")
(setf (get kortex_driver::PositionCommand :datatype-) "kortex_driver/PositionCommand")
(setf (get kortex_driver::PositionCommand :definition-)
      "
float32 position
float32 velocity
float32 acceleration
")



(provide :kortex_driver/PositionCommand "2a6d1bc721a1409197a45acce7ca57d7")


