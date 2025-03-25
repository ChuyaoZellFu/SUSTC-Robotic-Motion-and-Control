;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTLIMITATION")
  (make-package "KORTEX_DRIVER::JOINTLIMITATION"))

(in-package "ROS")
;;//! \htmlinclude JointLimitation.msg.html


(defclass kortex_driver::JointLimitation
  :super ros::object
  :slots (_joint_identifier _type _value ))

(defmethod kortex_driver::JointLimitation
  (:init
   (&key
    ((:joint_identifier __joint_identifier) 0)
    ((:type __type) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _joint_identifier (round __joint_identifier))
   (setq _type (round __type))
   (setq _value (float __value))
   self)
  (:joint_identifier
   (&optional __joint_identifier)
   (if __joint_identifier (setq _joint_identifier __joint_identifier)) _joint_identifier)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _joint_identifier
    4
    ;; uint32 _type
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _joint_identifier
       (write-long _joint_identifier s)
     ;; uint32 _type
       (write-long _type s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _joint_identifier
     (setq _joint_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::JointLimitation :md5sum-) "c7e8a86926cdf71a09bb6314d52e9586")
(setf (get kortex_driver::JointLimitation :datatype-) "kortex_driver/JointLimitation")
(setf (get kortex_driver::JointLimitation :definition-)
      "
uint32 joint_identifier
uint32 type
float32 value
")



(provide :kortex_driver/JointLimitation "c7e8a86926cdf71a09bb6314d52e9586")


