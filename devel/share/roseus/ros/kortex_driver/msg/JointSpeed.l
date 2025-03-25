;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointSpeed)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointSpeed (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTSPEED")
  (make-package "KORTEX_DRIVER::JOINTSPEED"))

(in-package "ROS")
;;//! \htmlinclude JointSpeed.msg.html


(defclass kortex_driver::JointSpeed
  :super ros::object
  :slots (_joint_identifier _value _duration ))

(defmethod kortex_driver::JointSpeed
  (:init
   (&key
    ((:joint_identifier __joint_identifier) 0)
    ((:value __value) 0.0)
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _joint_identifier (round __joint_identifier))
   (setq _value (float __value))
   (setq _duration (round __duration))
   self)
  (:joint_identifier
   (&optional __joint_identifier)
   (if __joint_identifier (setq _joint_identifier __joint_identifier)) _joint_identifier)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _joint_identifier
    4
    ;; float32 _value
    4
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _joint_identifier
       (write-long _joint_identifier s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _joint_identifier
     (setq _joint_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::JointSpeed :md5sum-) "fd26fce9b45cd51d86512ba0cf80d2bd")
(setf (get kortex_driver::JointSpeed :datatype-) "kortex_driver/JointSpeed")
(setf (get kortex_driver::JointSpeed :definition-)
      "
uint32 joint_identifier
float32 value
uint32 duration
")



(provide :kortex_driver/JointSpeed "fd26fce9b45cd51d86512ba0cf80d2bd")


